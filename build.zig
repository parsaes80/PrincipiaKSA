//! PrincipiaKSA's build, with Zig alone, for Linux, Windows and macOS from any of them.
//!
//! Principia's core and the parts of its dependencies it needs (extern/,
//! pinned git submodules) are compiled from source, as are protoc (for
//! Principia's serialization sources) and our C API; dotnet builds the C#
//! mod; the result is the mod folder, ready to use.
//!
//!   zig build          zig-out/PrincipiaKSA (the mod), zig-out/PrincipiaKSA.zip
//!                      and zig-out/windows-test (the native smoke test, for Windows)
//!   zig build test     the native smoke test, on this machine
//!   zig build game     ... and install the mod where KSA loads it
//!   zig build run      ... and start KSA through StarMap
//!   zig build update   move extern/ to the latest Principia and dependencies;
//!                      then build and test, and commit the new versions
//!   zig build sol-reference
//!                      the JPL check behind PrincipiaSystem's μ convention
//!
//!   -Dksa=<folder>      KSA's installation, for the C# references ($KSA_DIR)
//!   -Dstarmap=<folder>  StarMap's folder (else the one Borea uses, else KSA's
//!                       mods folder)
//!   -Dmods=<folder>     where `game` installs (else Borea's active instance,
//!                       else KSA's mods folder)
//!   -Dcs                `game` and `run` build and install only the C# mod,
//!                       keeping the native library already installed (after
//!                       a change to the C API, install everything once)

const std = @import("std");
const Build = std.Build;
const LazyPath = Build.LazyPath;

const Platform = enum { linux, windows, macos };
const platforms = [_]Platform{ .linux, .windows, .macos };

// The CPU features code is compiled for: the dependencies as Principia's
// install_deps.sh builds them (SSE4.1), Principia's core and CORE-MATH as
// its x64_AVX_FMA platform, abseil's AES-accelerated Randen with AES.
const Features = enum { base, avx_fma, aes };

pub fn build(b: *Build) !void {
    const ksa = b.option([]const u8, "ksa", "KSA's installation folder") orelse
        b.graph.environ_map.get("KSA_DIR") orelse findKsa(b);
    const starmap = b.option([]const u8, "starmap", "StarMap's folder (StarMap.API.dll)") orelse findStarMap(b);
    const mods = b.option([]const u8, "mods", "Where `zig build game` installs the mod") orelse "-";
    const csharp_only = b.option(bool, "cs", "`game` and `run`: build and install only the C# mod, keeping the native library installed") orelse false;

    try ensureSubmodules(b);
    const sources = try Sources.collect(b);

    // Principia's generated sources: its serialization protos, with protoc
    // built for this machine from the same protobuf, and its version.
    const host: Platform = switch (b.graph.host.result.os.tag) {
        .windows => .windows,
        .macos => .macos,
        else => .linux,
    };
    const protoc = buildProtoc(b, &sources, host);
    const run_protoc = b.addRunArtifact(protoc);
    run_protoc.addArg("-I");
    run_protoc.addDirectoryArg(b.path("extern/protobuf/src"));
    run_protoc.addArg("-I");
    run_protoc.addDirectoryArg(b.path("extern/Principia"));
    const generated = run_protoc.addPrefixedOutputDirectoryArg("--cpp_out=", "generated");
    for (sources.protos) |proto| run_protoc.addFileArg(b.path(b.fmt("extern/Principia/{s}", .{proto})));
    const version = b.addWriteFiles().add("base/version.generated.cc", try principiaVersion(b));

    var dependencies: [platforms.len]Dependencies = undefined;
    var libraries: [platforms.len]*Build.Step.Compile = undefined;
    var tests: [platforms.len]*Build.Step.Compile = undefined;
    for (platforms, 0..) |platform, i| {
        const deps = Dependencies.build(b, &sources, platform);
        dependencies[i] = deps;
        libraries[i] = buildLibrary(b, &sources, platform, deps, generated, version);
        tests[i] = buildSmokeTest(b, platform, libraries[i]);
    }
    const linux_lib = libraries[0];
    const windows_lib = libraries[1];
    const macos_lib = libraries[2];

    // The C# mod, against KSA's and StarMap's assemblies.
    const dotnet = b.addSystemCommand(&.{ "dotnet", "build" });
    dotnet.addFileArg(b.path("src/PrincipiaKSA/PrincipiaKSA.csproj"));
    dotnet.addArgs(&.{ "-c", "Release", "-nologo", "-v", "quiet" });
    dotnet.addArg(b.fmt("-p:KsaDir={s}", .{ksa}));
    dotnet.addArg(b.fmt("-p:StarMapDir={s}", .{starmap}));
    dotnet.addArg("-o");
    const csharp = dotnet.addOutputDirectoryArg("csharp");
    dotnet.has_side_effects = true; // dotnet's own build is incremental.
    if (!hasFile(b, ksa, "KSA.dll"))
        dotnet.step.dependOn(&b.addFail(b.fmt("KSA not found at '{s}': pass -Dksa=<folder> or set KSA_DIR", .{ksa})).step);
    if (!hasFile(b, starmap, "StarMap.API.dll"))
        dotnet.step.dependOn(&b.addFail(b.fmt("StarMap.API.dll not found in '{s}': pass -Dstarmap=<folder>", .{starmap})).step);

    // This repository's version, for the package.
    const describe = b.addSystemCommand(&.{ "git", "describe", "--tags", "--always", "--dirty" });
    describe.setCwd(b.path("."));
    describe.has_side_effects = true;
    const mod_version = describe.captureStdOut(.{});

    // The mod folder, as KSA loads it.
    const files = [_]struct { []const u8, LazyPath }{
        .{ "PrincipiaKSA.dll", csharp.path(b, "PrincipiaKSA.dll") },
        .{ "mod.toml", b.path("src/PrincipiaKSA/mod.toml") },
        .{ "libprincipia_ksa_AVX.so", linux_lib.getEmittedBin() },
        .{ "principia_ksa_AVX.dll", windows_lib.getEmittedBin() },
        .{ "libprincipia_ksa_arm64.dylib", macos_lib.getEmittedBin() },
        .{ "sol_gravity_model.proto.txt", b.path("extern/Principia/astronomy/sol_gravity_model.proto.txt") },
        .{ "README.txt", b.path("packaging/README.txt") },
        .{ "VERSION", mod_version },
        .{ "licenses/PrincipiaKSA.txt", b.path("LICENSE") },
        .{ "licenses/Principia.txt", b.path("extern/Principia/LICENSE.txt") },
        .{ "licenses/abseil.txt", b.path("extern/abseil-cpp/LICENSE") },
        .{ "licenses/protobuf.txt", b.path("extern/protobuf/LICENSE") },
        .{ "licenses/re2.txt", b.path("extern/re2/LICENSE") },
        .{ "licenses/gipfeli.txt", b.path("extern/gipfeli/COPYING") },
        .{ "licenses/zfp.txt", b.path("extern/zfp/LICENSE") },
        .{ "licenses/boost.txt", b.path("extern/multiprecision/LICENSE") },
        .{ "licenses/core-math.txt", b.path("packaging/core-math-LICENSE.txt") },
        .{ "licenses/simde.txt", b.path("extern/simde/COPYING") },
        .{ "licenses/libc++.txt", zigLib(b, "libcxx/LICENSE.TXT") },
        .{ "licenses/mingw-w64.txt", zigLib(b, "libc/mingw/COPYING") },
    };
    for (files) |file| b.getInstallStep().dependOn(
        &b.addInstallFileWithDir(file[1], .{ .custom = "PrincipiaKSA" }, file[0]).step,
    );

    // The same, zipped.
    const zip = b.addRunArtifact(tool(b, "zip"));
    const zip_file = zip.addOutputFileArg("PrincipiaKSA.zip");
    for (files) |file| {
        zip.addArg(b.fmt("PrincipiaKSA/{s}", .{file[0]}));
        zip.addFileArg(file[1]);
    }
    b.getInstallStep().dependOn(&b.addInstallFile(zip_file, "PrincipiaKSA.zip").step);

    // The native smoke test for Windows, to run there.
    const windows_test = [_]struct { []const u8, LazyPath }{
        .{ "smoke_test.exe", tests[1].getEmittedBin() },
        .{ "principia_ksa_AVX.dll", windows_lib.getEmittedBin() },
        .{ "sol_gravity_model.proto.txt", b.path("extern/Principia/astronomy/sol_gravity_model.proto.txt") },
        .{ "run_smoke_test.bat", b.path("packaging/run_smoke_test.bat") },
    };
    for (windows_test) |file| b.getInstallStep().dependOn(
        &b.addInstallFileWithDir(file[1], .{ .custom = "windows-test" }, file[0]).step,
    );

    // zig build test: the smoke test on this machine.
    const test_step = b.step("test", "Run the native smoke test on this machine");
    const smoke = b.addRunArtifact(tests[@backingInt(host)]);
    smoke.addFileArg(b.path("extern/Principia/astronomy/sol_gravity_model.proto.txt"));
    smoke.stdio = .inherit; // Fails on a nonzero exit.
    smoke.has_side_effects = true;
    test_step.dependOn(&smoke.step);

    // zig build sol-reference: which μ convention turns KSA's elements back
    // into the real solar system (native/tools/sol_reference.cpp), against
    // Principia's own JPL initial state. Reuses the library's objects.
    const reference_module = principiaCore(b, &sources, host, dependencies[@backingInt(host)], generated, version);
    reference_module.addCSourceFile(.{ .file = b.path("native/tools/sol_reference.cpp"), .flags = principiaFlags(b, reference_module.resolved_target.?) });
    const reference = b.addRunArtifact(b.addExecutable(.{ .name = "sol_reference", .root_module = reference_module }));
    reference.addDirectoryArg(b.path("extern/Principia"));
    reference.stdio = .inherit;
    reference.has_side_effects = true;
    b.step("sol-reference", "Compare KSA's planets' elements with Principia's JPL ephemeris").dependOn(&reference.step);

    // zig build game / run: into the game, and start it. With -Dcs,
    // nothing native is built: the first two files are the C# mod's. The
    // C# debug symbols go too, for line numbers in the log's stack traces.
    const game_files = if (csharp_only) files[0..2] else files[0..];
    for ([_][]const u8{ "game", "run" }) |action| {
        const game = b.addRunArtifact(tool(b, "game"));
        game.addArgs(&.{ action, mods, ksa });
        for (game_files) |file| {
            game.addArg(file[0]);
            game.addFileArg(file[1]);
        }
        game.addArg("PrincipiaKSA.pdb");
        game.addFileArg(csharp.path(b, "PrincipiaKSA.pdb"));
        game.has_side_effects = true;
        game.stdio = .inherit;
        const step = if (std.mem.eql(u8, action, "game"))
            b.step("game", "Build and install the mod where KSA loads it")
        else
            b.step("run", "Build, install and start KSA through StarMap");
        step.dependOn(&game.step);
    }

    // zig build update: the latest Principia and dependencies.
    const update = b.addSystemCommand(&.{ "git", "submodule", "update", "--init", "--remote" });
    update.setCwd(b.path("."));
    update.has_side_effects = true;
    update.stdio = .inherit;
    b.step("update", "Move extern/ to the latest Principia and dependencies").dependOn(&update.step);
}

fn target(b: *Build, platform: Platform, features: Features) Build.ResolvedTarget {
    const x86 = std.Target.x86;
    const added = switch (features) {
        .base => x86.featureSet(&.{.sse4_1}),
        .avx_fma => x86.featureSet(&.{ .sse4_1, .avx, .fma }),
        .aes => x86.featureSet(&.{ .sse4_1, .aes }),
    };
    return switch (platform) {
        // glibc 2.28 (2018) and later.
        .linux => b.resolveTargetQuery(.{
            .cpu_arch = .x86_64,
            .os_tag = .linux,
            .abi = .gnu,
            .glibc_version = .{ .major = 2, .minor = 28, .patch = 0 },
            .cpu_features_add = added,
        }),
        .windows => b.resolveTargetQuery(.{
            .cpu_arch = .x86_64,
            .os_tag = .windows,
            .abi = .gnu,
            .cpu_features_add = added,
        }),
        .macos => b.resolveTargetQuery(.{
            .cpu_arch = .aarch64,
            .os_tag = .macos,
            .cpu_model = .{ .explicit = &std.Target.aarch64.cpu.apple_m1 },
            .os_version_min = .{ .semver = .{ .major = 14, .minor = 0, .patch = 0 } },
        }),
    };
}

fn isWindows(t: Build.ResolvedTarget) bool {
    return t.result.os.tag == .windows;
}

fn isMacos(t: Build.ResolvedTarget) bool {
    return t.result.os.tag == .macos;
}

// Native code is always optimized, unsanitized and stripped.
fn nativeModule(b: *Build, t: Build.ResolvedTarget) *Build.Module {
    const m = b.createModule(.{
        .target = t,
        .optimize = .ReleaseFast,
        .link_libc = true,
        .link_libcpp = true,
        .pic = true,
        .strip = true,
        .sanitize_c = .off,
        .omit_frame_pointer = false,
    });
    if (isMacos(t)) {
        m.addIncludePath(b.path("native/macos/include"));
        m.addSystemIncludePath(b.path("extern/simde"));
        m.addFrameworkPath(b.path("native/macos"));
    }
    return m;
}

// macOS: hidden symbols stand in for exports.map.
fn macosFlags(b: *Build) []const []const u8 {
    return b.allocator.dupe([]const u8, &.{
        "-include", rootPath(b, "native/macos/include/float_first.h"), "-fvisibility=hidden",
    }) catch @panic("OOM");
}

fn platformFlags(b: *Build, t: Build.ResolvedTarget, flags: []const []const u8) []const []const u8 {
    if (!isMacos(t)) return flags;
    return std.mem.concat(b.allocator, []const u8, &.{ flags, macosFlags(b) }) catch @panic("OOM");
}

// The dependencies, as Principia's install_deps.sh builds them.
const dep_cflags = [_][]const u8{ "-O3", "-DNDEBUG", "-fno-omit-frame-pointer", "-w" };
const dep_cxxflags = [_][]const u8{"-std=c++20"} ++ dep_cflags;

// Principia's own flags (its Makefile's SHARED_ARGS and x64_AVX_FMA).
const principia_common_flags = [_][]const u8{
    "-std=c++23",
    "-O3",
    "-fbracket-depth=257",
    "-fdeclspec",
    "-fexceptions",
    "-ffp-contract=off",
    "-fno-omit-frame-pointer",
    "-fno-char8_t",
    "-DNDEBUG",
    "-DPRINCIPIA_REQUIRES_AVX=1",
    "-DPRINCIPIA_REQUIRES_FMA=1",
    "-DPROJECT_DIR=std::filesystem::path(\"ksp_plugin_adapter/\")",
    "-DSOLUTION_DIR=std::filesystem::path(\"./\")",
    "-Wall",
    "-Wno-unknown-pragmas",
    "-Wno-deprecated-declarations",
    "-Wno-mathematical-notation-identifier-extension",
    "-Wno-c++23-extensions",
    "--system-header-prefix=serialization/",
};
const principia_x64_flags = principia_common_flags ++ [_][]const u8{"-DPLATFORM_WITH_CPU_FEATURES=\"x64_AVX_FMA\""};
const principia_linux_flags = principia_x64_flags ++ [_][]const u8{"-DTEMP_DIR=std::filesystem::path(\"/tmp\")"};
const principia_windows_flags = principia_x64_flags ++ [_][]const u8{"-DTEMP_DIR=std::filesystem::path(\"C:/Windows/Temp\")"};
const principia_macos_flags = principia_common_flags ++ [_][]const u8{
    "-DPLATFORM_WITH_CPU_FEATURES=\"arm64\"",
    "-DTEMP_DIR=std::filesystem::path(\"/tmp\")",
};

fn principiaFlags(b: *Build, t: Build.ResolvedTarget) []const []const u8 {
    if (isWindows(t)) return &principia_windows_flags;
    if (isMacos(t)) return platformFlags(b, t, &principia_macos_flags);
    return &principia_linux_flags;
}

// Principia's base/macros.hpp with an arm64 case, used before Principia's own.
fn arm64Macros(b: *Build) LazyPath {
    const path = "extern/Principia/base/macros.hpp";
    b.dependOnFileContents(b.path(path));
    const text = readFile(b, path) catch @panic("cannot read " ++ path);
    const x64 = std.mem.indexOf(u8, text, "char const* const Architecture = \"x86-64\";");
    const newline = if (x64) |at| std.mem.indexOfPos(u8, text, at, "\n#else") else null;
    const problem: ?[]const u8 = if (newline == null)
        "Principia's base/macros.hpp changed: no x86-64 case to add arm64 after (arm64Macros in build.zig)"
    else if (std.mem.indexOf(u8, text[newline.?..], "aarch64") != null)
        "Principia's base/macros.hpp has an arm64 case of its own: remove arm64Macros from build.zig"
    else
        null;
    if (problem) |message| {
        const files = b.addWriteFiles();
        _ = files.add("base/macros.hpp", b.fmt("#error \"{s}\"\n", .{message}));
        return files.getDirectory();
    }
    const split = newline.? + 1;
    const patched = b.fmt("{s}{s}{s}", .{
        text[0..split],
        \\#elif defined(__aarch64__)
        \\#define ARCH_CPU_ARM_FAMILY 1
        \\#define ARCH_CPU_ARM64 1
        \\#define ARCH_CPU_64_BITS 1
        \\#define ARCH_CPU_LITTLE_ENDIAN 1
        \\char const* const Architecture = "arm64";
        \\
        ,
        text[split..],
    });
    const files = b.addWriteFiles();
    _ = files.add("base/macros.hpp", patched);
    return files.getDirectory();
}

fn principiaIncludes(b: *Build, m: *Build.Module, generated: LazyPath) void {
    if (m.resolved_target.?.result.cpu.arch == .aarch64) m.addIncludePath(arm64Macros(b));
    m.addIncludePath(b.path("extern/Principia"));
    m.addIncludePath(generated);
    for ([_][]const u8{
        "extern/abseil-cpp",      "extern/protobuf/src",   "extern/protobuf/third_party/utf8_range",
        "extern/gipfeli/include", "extern/re2",            "extern/core-math/include",
        "extern/zfp/include",     "extern/config/include", "extern/multiprecision/include",
    }) |dir| m.addSystemIncludePath(b.path(dir));
}

fn staticLibrary(
    b: *Build,
    t: Build.ResolvedTarget,
    name: []const u8,
    files: []const []const u8,
    flags: []const []const u8,
    includes: []const []const u8,
    language: ?std.Build.Module.CSourceLanguage,
) *Build.Step.Compile {
    const m = nativeModule(b, t);
    for (includes) |dir| m.addIncludePath(b.path(dir));
    m.addCSourceFiles(.{ .root = b.path("extern"), .files = files, .flags = platformFlags(b, t, flags), .language = language });
    return b.addLibrary(.{ .name = name, .linkage = .static, .root_module = m });
}

const absl_includes = [_][]const u8{"extern/abseil-cpp"};
const protobuf_includes = [_][]const u8{
    "extern/abseil-cpp", "extern/protobuf/src",                  "extern/protobuf/third_party/utf8_range",
    "extern/protobuf",   "extern/protobuf/upb/reflection/cmake",
};

const Dependencies = struct {
    libraries: []const *Build.Step.Compile,

    fn build(b: *Build, s: *const Sources, platform: Platform) Dependencies {
        const t = target(b, platform, .base);
        const absl = staticLibrary(b, t, "absl", if (platform == .windows) s.absl else s.absl_posix, &(dep_cxxflags ++ [_][]const u8{"-DNOMINMAX"}), &absl_includes, null);
        // AES for randen_hwaes, as abseil's own build.
        const hwaes = staticLibrary(b, target(b, platform, .aes), "absl_hwaes", &.{"abseil-cpp/absl/random/internal/randen_hwaes.cc"}, &(dep_cxxflags ++ [_][]const u8{"-DNOMINMAX"}), &absl_includes, null);
        const protobuf = staticLibrary(b, t, "protobuf", s.protobuf, &(dep_cxxflags ++ [_][]const u8{"-DGOOGLE_PROTOBUF_CMAKE_BUILD"}), &protobuf_includes, null);
        const utf8 = staticLibrary(b, t, "utf8_range", &.{"protobuf/third_party/utf8_range/utf8_range.c"}, &dep_cflags, &.{"extern/protobuf/third_party/utf8_range"}, null);
        const gipfeli = staticLibrary(b, t, "gipfeli", s.gipfeli, &(dep_cxxflags ++ [_][]const u8{"-DGIPFELI_NO_UNALIGNED"}), &.{"extern/gipfeli/include"}, null);
        const re2 = staticLibrary(b, t, "re2", &re2_sources, &dep_cxxflags, &.{ "extern/abseil-cpp", "extern/re2" }, null);
        // CORE-MATH (the slow path of Principia's sin and cos) counts the
        // leading zeros of 64-bit words with __builtin_clzl, whose unsigned
        // long is 32 bits on Windows (only MSVC gets its fallback): it
        // returned sines of 64 there. Its exact fma is the FMA instruction.
        const core_math = staticLibrary(b, target(b, platform, .avx_fma), "core-math", &.{ "core-math/src/binary64/cos/cos.c", "core-math/src/binary64/sin/sin.c" }, &(dep_cxxflags ++ [_][]const u8{ "-D__builtin_clzl=__builtin_clzll", "-ffp-contract=off" }), &.{ "extern/core-math/include", "extern/abseil-cpp" }, .cpp);
        const zfp = staticLibrary(b, t, "zfp", &zfp_sources, &dep_cflags, &.{"extern/zfp/include"}, null);
        return .{ .libraries = b.allocator.dupe(*Build.Step.Compile, &.{ absl, hwaes, protobuf, utf8, gipfeli, re2, core_math, zfp }) catch @panic("OOM") };
    }

    fn linkInto(d: Dependencies, m: *Build.Module) void {
        for (d.libraries) |library| m.linkLibrary(library);
        if (isMacos(m.resolved_target.?)) m.linkFramework("CoreFoundation", .{});
    }
};

const re2_sources = blk: {
    const files = [_][]const u8{
        "re2/bitmap256.cc",    "re2/bitstate.cc",    "re2/compile.cc",          "re2/dfa.cc",
        "re2/filtered_re2.cc", "re2/mimics_pcre.cc", "re2/nfa.cc",              "re2/onepass.cc",
        "re2/parse.cc",        "re2/perl_groups.cc", "re2/prefilter.cc",        "re2/prefilter_tree.cc",
        "re2/prog.cc",         "re2/re2.cc",         "re2/regexp.cc",           "re2/set.cc",
        "re2/simplify.cc",     "re2/tostring.cc",    "re2/unicode_casefold.cc", "re2/unicode_groups.cc",
        "util/rune.cc",        "util/strutil.cc",
    };
    var paths: [files.len][]const u8 = undefined;
    for (files, 0..) |file, i| paths[i] = std.fmt.comptimePrint("re2/{s}", .{file});
    break :blk paths;
};

const zfp_sources = blk: {
    var paths: [2 + 4 * 4 * 2][]const u8 = undefined;
    paths[0] = "zfp/src/zfp.c";
    paths[1] = "zfp/src/bitstream.c";
    var i = 2;
    for ("1234") |d| {
        for ("fdil") |kind| {
            paths[i] = std.fmt.comptimePrint("zfp/src/encode{c}{c}.c", .{ d, kind });
            paths[i + 1] = std.fmt.comptimePrint("zfp/src/decode{c}{c}.c", .{ d, kind });
            i += 2;
        }
    }
    break :blk paths;
};

fn buildProtoc(b: *Build, s: *const Sources, platform: Platform) *Build.Step.Compile {
    const t = target(b, platform, .base);
    const m = nativeModule(b, t);
    for (protobuf_includes) |dir| m.addIncludePath(b.path(dir));
    const cxxflags = platformFlags(b, t, &(dep_cxxflags ++ [_][]const u8{"-DGOOGLE_PROTOBUF_CMAKE_BUILD"}));
    m.addCSourceFiles(.{ .root = b.path("extern"), .files = s.protoc, .flags = cxxflags });
    m.addCSourceFiles(.{ .root = b.path("extern"), .files = s.upb, .flags = platformFlags(b, t, &dep_cflags) });
    m.addCSourceFiles(.{ .root = b.path("extern"), .files = &.{"protobuf/src/google/protobuf/compiler/main.cc"}, .flags = cxxflags });
    Dependencies.build(b, s, platform).linkInto(m);
    return b.addExecutable(.{ .name = "protoc", .root_module = m });
}

// Principia's core (astronomy, base, geometry, numerics, physics and the
// serialization sources) with its dependencies, for the AVX/FMA target.
fn principiaCore(
    b: *Build,
    s: *const Sources,
    platform: Platform,
    deps: Dependencies,
    generated: LazyPath,
    version: LazyPath,
) *Build.Module {
    const t = target(b, platform, .avx_fma);
    const m = nativeModule(b, t);
    const flags = principiaFlags(b, t);
    principiaIncludes(b, m, generated);
    m.addCSourceFiles(.{ .root = b.path("extern/Principia"), .files = s.principia, .flags = flags });
    for (s.protos) |proto| m.addCSourceFile(.{
        .file = generated.path(b, b.fmt("{s}.pb.cc", .{proto[0 .. proto.len - ".proto".len]})),
        .flags = flags,
    });
    m.addCSourceFile(.{ .file = version, .flags = flags });
    deps.linkInto(m);
    if (isWindows(t)) {
        m.linkSystemLibrary("dbghelp", .{});
        m.linkSystemLibrary("bcrypt", .{});
    } else {
        m.linkSystemLibrary("pthread", .{});
    }
    return m;
}

// libprincipia_ksa_AVX.so / principia_ksa_AVX.dll: Principia's core, its
// generated sources and our C API, exporting only principia_ksa__*.
fn buildLibrary(
    b: *Build,
    s: *const Sources,
    platform: Platform,
    deps: Dependencies,
    generated: LazyPath,
    version: LazyPath,
) *Build.Step.Compile {
    const m = principiaCore(b, s, platform, deps, generated, version);
    const t = m.resolved_target.?;
    m.addCSourceFile(.{
        .file = b.path("native/principia_ksa.cpp"),
        .flags = std.mem.concat(b.allocator, []const u8, &.{ principiaFlags(b, t), &.{"-DPRINCIPIA_KSA_BUILD"} }) catch @panic("OOM"),
    });
    const name = if (isMacos(t)) "principia_ksa_arm64" else "principia_ksa_AVX";
    const library = b.addLibrary(.{ .name = name, .linkage = .dynamic, .root_module = m });
    if (isMacos(t)) {
        library.link_gc_sections = true;
    } else if (!isWindows(t)) {
        library.setVersionScript(b.path("native/exports.map"));
        library.link_gc_sections = true;
    }
    return library;
}

fn buildSmokeTest(b: *Build, platform: Platform, library: *Build.Step.Compile) *Build.Step.Compile {
    const t = target(b, platform, .base);
    const m = nativeModule(b, t);
    m.addIncludePath(b.path("native"));
    m.addCSourceFile(.{ .file = b.path("native/test/smoke_test.cpp"), .flags = platformFlags(b, t, &.{ "-std=c++20", "-O1", "-Wall" }) });
    m.linkLibrary(library);
    if (isMacos(t)) {
        m.addRPathSpecial("@loader_path");
        m.addRPath(library.getEmittedBinDirectory());
    } else if (!isWindows(t)) {
        m.addRPathSpecial("$ORIGIN");
        m.addRPath(library.getEmittedBinDirectory());
    }
    return b.addExecutable(.{ .name = "smoke_test", .root_module = m });
}

// The build's own tools (tools/*.zig), for this machine.
fn tool(b: *Build, name: []const u8) *Build.Step.Compile {
    return b.addExecutable(.{
        .name = name,
        .root_module = b.createModule(.{
            .root_source_file = b.path(b.fmt("tools/{s}.zig", .{name})),
            .target = b.graph.host,
            .optimize = .ReleaseSafe,
        }),
    });
}

fn zigLib(b: *Build, sub_path: []const u8) LazyPath {
    _ = b;
    return .{ .relative = .{ .base = .zig_lib, .sub_path = sub_path } };
}

// The sources, from the pinned checkouts: listed by the dependencies' own
// builds where they have lists, else by directory. Configuration is cached;
// it is redone when a checkout moves to another commit.
const Sources = struct {
    absl: []const []const u8,
    absl_posix: []const []const u8,
    protobuf: []const []const u8,
    protoc: []const []const u8,
    upb: []const []const u8,
    gipfeli: []const []const u8,
    principia: []const []const u8,
    protos: []const []const u8,

    fn collect(b: *Build) !Sources {
        for ([_][]const u8{ "Principia", "abseil-cpp", "protobuf", "re2", "gipfeli", "core-math", "zfp", "config", "multiprecision", "simde" }) |name|
            try dependOnCheckout(b, name);

        var absl: std.ArrayList([]const u8) = .empty;
        var absl_posix: std.ArrayList([]const u8) = .empty;
        const excluded = [_][]const u8{ "_test", "benchmark", "test_", "testutil", "testing", "_mock", "matchers", "gentables", "print_hash_of", "_fuzz", "/flags/", "randen_hwaes.cc" };
        for (try listFiles(b, "extern/abseil-cpp/absl", ".cc", true)) |file| {
            if (containsAny(file, &excluded)) continue;
            const path = b.fmt("abseil-cpp/absl/{s}", .{file});
            try absl.append(b.allocator, path);
            if (!std.mem.endsWith(u8, file, "_win.cc")) try absl_posix.append(b.allocator, path); // windows.h
        }

        const lists = try readFile(b, "extern/protobuf/src/file_lists.cmake");

        var gipfeli: std.ArrayList([]const u8) = .empty;
        for (try listFiles(b, "extern/gipfeli/src", ".cc", false)) |file| {
            if (!std.mem.endsWith(u8, file, "_test.cc")) try gipfeli.append(b.allocator, b.fmt("gipfeli/src/{s}", .{file}));
        }

        // Principia's core: what Ephemeris needs, without its KSP plugin,
        // journal or tests.
        var principia: std.ArrayList([]const u8) = .empty;
        for ([_][]const u8{ "astronomy", "base", "geometry", "numerics", "physics" }) |dir| {
            for (try listFiles(b, b.fmt("extern/Principia/{s}", .{dir}), ".cpp", false)) |file| {
                if (std.mem.endsWith(u8, file, "_test.cpp") or std.mem.startsWith(u8, file, "fake_") or
                    std.mem.startsWith(u8, file, "mock_") or std.mem.eql(u8, file, "benchmark.cpp")) continue;
                try principia.append(b.allocator, b.fmt("{s}/{s}", .{ dir, file }));
            }
        }
        var protos: std.ArrayList([]const u8) = .empty;
        for (try listFiles(b, "extern/Principia/serialization", ".proto", false)) |file|
            try protos.append(b.allocator, b.fmt("serialization/{s}", .{file}));

        var upb = try cmakeList(b, lists, "libupb_srcs");
        try upb.append(b.allocator, "protobuf/upb/reflection/cmake/google/protobuf/descriptor.upb_minitable.c");

        return .{
            .absl = absl.items,
            .absl_posix = absl_posix.items,
            .protobuf = (try cmakeList(b, lists, "libprotobuf_srcs")).items,
            .protoc = (try cmakeList(b, lists, "libprotoc_srcs")).items,
            .upb = upb.items,
            .gipfeli = gipfeli.items,
            .principia = principia.items,
            .protos = protos.items,
        };
    }
};

fn containsAny(haystack: []const u8, needles: []const []const u8) bool {
    for (needles) |needle| if (std.mem.indexOf(u8, haystack, needle) != null) return true;
    return false;
}

// The files with `extension` under `dir` (build root relative), sorted,
// relative to it with '/' separators.
fn listFiles(b: *Build, dir: []const u8, extension: []const u8, recursive: bool) ![]const []const u8 {
    const io = b.graph.io;
    var handle = try b.root.openDir(io, dir, .{ .iterate = true });
    defer handle.close(io);
    var files: std.ArrayList([]const u8) = .empty;
    if (recursive) {
        var walker = try handle.walk(b.allocator);
        defer walker.deinit();
        while (try walker.next(io)) |entry| {
            if (entry.kind != .file or !std.mem.endsWith(u8, entry.path, extension)) continue;
            const path = try b.allocator.dupe(u8, entry.path);
            std.mem.replaceScalar(u8, path, '\\', '/');
            try files.append(b.allocator, path);
        }
    } else {
        var it = handle.iterate();
        while (try it.next(io)) |entry| {
            if (entry.kind != .file or !std.mem.endsWith(u8, entry.name, extension)) continue;
            try files.append(b.allocator, try b.allocator.dupe(u8, entry.name));
        }
    }
    std.mem.sort([]const u8, files.items, {}, struct {
        fn lessThan(_: void, x: []const u8, y: []const u8) bool {
            return std.mem.lessThan(u8, x, y);
        }
    }.lessThan);
    return files.items;
}

fn readFile(b: *Build, path: []const u8) ![]u8 {
    const file = try b.root.join(b.allocator, path);
    return file.root_dir.handle.readFileAlloc(b.graph.io, file.sub_path, b.allocator, .unlimited);
}

fn rootPath(b: *Build, sub_path: []const u8) []const u8 {
    return b.root.joinString(b.allocator, sub_path) catch @panic("OOM");
}

// A `set(name ...)` list of protobuf's file_lists.cmake, relative to extern/.
fn cmakeList(b: *Build, text: []const u8, name: []const u8) !std.ArrayList([]const u8) {
    const start = std.mem.indexOf(u8, text, b.fmt("set({s}\n", .{name})) orelse return error.ProtobufListMissing;
    var lines = std.mem.splitScalar(u8, text[start..], '\n');
    _ = lines.next();
    var files: std.ArrayList([]const u8) = .empty;
    while (lines.next()) |line| {
        const entry = std.mem.trim(u8, line, " \t\r");
        if (std.mem.startsWith(u8, entry, ")")) break;
        const prefix = "${protobuf_SOURCE_DIR}/";
        if (std.mem.startsWith(u8, entry, prefix))
            try files.append(b.allocator, b.fmt("protobuf/{s}", .{entry[prefix.len..]}));
    }
    return files;
}

// Redo the configuration when extern/<name> moves to another commit.
fn dependOnCheckout(b: *Build, name: []const u8) !void {
    const dot_git = b.fmt("extern/{s}/.git", .{name});
    if (exists(b, b.fmt("{s}/HEAD", .{dot_git}))) { // A checkout with its own .git directory.
        b.dependOnFileContents(b.path(b.fmt("{s}/HEAD", .{dot_git})));
        return;
    }
    const text = try readFile(b, dot_git);
    // "gitdir: <path>", relative to the checkout: a submodule's git directory.
    const git_dir = std.mem.trim(u8, text["gitdir:".len..], " \r\n");
    if (std.fs.path.isAbsolute(git_dir)) {
        b.dependOnFileContents(.{ .cwd_relative = b.fmt("{s}/HEAD", .{git_dir}) });
    } else {
        const resolved = try std.fs.path.resolveAllocPosix(b.allocator, &.{ b.fmt("extern/{s}", .{name}), git_dir, "HEAD" });
        b.dependOnFileContents(b.path(resolved));
    }
}

fn ensureSubmodules(b: *Build) !void {
    if (exists(b, "extern/Principia/physics")) {
        if (exists(b, "extern/simde/simde")) return;
        std.log.info("fetching SIMDe (git submodule update --init extern/simde)", .{});
        _ = b.run(&.{ "git", "-C", rootPath(b, "."), "submodule", "update", "--init", "extern/simde" });
        return;
    }
    std.log.info("fetching Principia and its dependencies (git submodule update --init)", .{});
    _ = b.run(&.{ "git", "-C", rootPath(b, "."), "submodule", "update", "--init" });
}

// Principia's base/version.generated.cc, as its generate_version_translation_unit.sh.
fn principiaVersion(b: *Build) ![]const u8 {
    const dir = rootPath(b, "extern/Principia");
    const version = std.mem.trim(u8, b.run(&.{ "git", "-C", dir, "describe", "--tags", "--always", "--dirty", "--abbrev=40", "--long" }), " \r\n");
    const seconds = try std.fmt.parseInt(u64, std.mem.trim(u8, b.run(&.{ "git", "-C", dir, "log", "-1", "--format=%ct" }), " \r\n"), 10);
    const epoch: std.time.epoch.EpochSeconds = .{ .secs = seconds };
    const year_day = epoch.getEpochDay().calculateYearDay();
    const month_day = year_day.calculateMonthDay();
    const day_seconds = epoch.getDaySeconds();
    return b.fmt(
        "\xef\xbb\xbf\n#include \"base/version.hpp\"\n\nnamespace principia {{\nnamespace base {{\nnamespace _version {{\n" ++
            "namespace internal {{\n\nchar const BuildDate[] = \"{d:0>4}-{d:0>2}-{d:0>2}T{d:0>2}:{d:0>2}:{d:0>2}Z\";\n" ++
            "char const Version[] =\n    u8\"{s}\";\n\n}}  // namespace internal\n}}  // namespace _version\n" ++
            "}}  // namespace base\n}}  // namespace principia\n",
        .{
            year_day.year,                 month_day.month.numeric(),        month_day.day_index + 1,
            day_seconds.getHoursIntoDay(), day_seconds.getMinutesIntoHour(), day_seconds.getSecondsIntoMinute(),
            version,
        },
    );
}

fn exists(b: *Build, path: []const u8) bool {
    const io = b.graph.io;
    if (std.fs.path.isAbsolute(path)) {
        std.Io.Dir.cwd().access(io, path, .{}) catch return false;
        return true;
    }
    b.root.access(io, path, .{}) catch return false;
    return true;
}

fn home(b: *Build) []const u8 {
    return b.graph.environ_map.get("HOME") orelse b.graph.environ_map.get("USERPROFILE") orelse ".";
}

// The Borea launcher's folder.
fn boreaDir(b: *Build) []const u8 {
    return if (b.graph.host.result.os.tag == .macos)
        b.fmt("{s}/Library/Application Support/Borea", .{home(b)})
    else
        b.fmt("{s}/.local/share/Borea", .{home(b)});
}

// Whether `dir` holds `file`. The configure phase is cached: what it found
// is tracked, so a change (a launcher updating StarMap) configures again.
fn hasFile(b: *Build, dir: []const u8, file: []const u8) bool {
    const path = b.fmt("{s}/{s}", .{ dir, file });
    b.dependOnFileMetadata(.{ .cwd_relative = path });
    return exists(b, path);
}

// A string in the Borea launcher's settings: `key = "value"` in `[section]`
// (or before any section if it is ""). Tracked as above.
fn boreaSetting(b: *Build, section: []const u8, key: []const u8) ?[]const u8 {
    const path = b.fmt("{s}/borea-settings.toml", .{boreaDir(b)});
    b.dependOnFileContents(.{ .cwd_relative = path });
    const text = std.Io.Dir.cwd().readFileAlloc(b.graph.io, path, b.allocator, .limited(1 << 20)) catch return null;
    var in_section = section.len == 0;
    var lines = std.mem.splitScalar(u8, text, '\n');
    while (lines.next()) |line| {
        const trimmed = std.mem.trim(u8, line, " \t\r");
        if (trimmed.len > 0 and trimmed[0] == '[') {
            in_section = std.mem.eql(u8, trimmed, b.fmt("[{s}]", .{section}));
            continue;
        }
        if (!in_section or !std.mem.startsWith(u8, trimmed, key)) continue;
        const rest = std.mem.trim(u8, trimmed[key.len..], " \t");
        if (rest.len == 0 or rest[0] != '=') continue;
        const value = std.mem.trim(u8, rest[1..], " \t");
        if (value.len < 2 or value[0] != '"' or value[value.len - 1] != '"') return null;
        // TOML escapes Windows' backslashes.
        const unescaped = std.mem.replaceOwned(u8, b.allocator, value[1 .. value.len - 1], "\\\\", "\\") catch
            @panic("OOM");
        return unescaped;
    }
    return null;
}

// KSA: the Borea launcher's game folder, else its usual installations.
fn findKsa(b: *Build) []const u8 {
    if (boreaSetting(b, "", "GameDirectoryPath")) |dir| {
        if (hasFile(b, dir, "KSA.dll")) return dir;
    }
    const candidates = [_][]const u8{
        "C:/Program Files/Kitten Space Agency",
        "/Applications/Kitten Space Agency.app/Contents/Resources",
        b.fmt("{s}/Applications/Kitten Space Agency.app/Contents/Resources", .{home(b)}),
        b.fmt("{s}/Games/Ahwoo/Kitten Space Agency", .{home(b)}),
    };
    for (candidates) |dir| if (hasFile(b, dir, "KSA.dll")) return dir;
    return candidates[candidates.len - 1];
}

// StarMap: the one the Borea launcher is set to use, else in KSA's mods
// folder.
fn findStarMap(b: *Build) []const u8 {
    if (boreaSetting(b, "LoaderInstallations.StarMap", "DirectoryPath")) |dir| {
        if (hasFile(b, dir, "StarMap.API.dll")) return dir;
    }
    const mods = b.fmt("{s}/Documents/My Games/Kitten Space Agency/mods/StarMap", .{home(b)});
    _ = hasFile(b, mods, "StarMap.API.dll");
    return mods;
}
