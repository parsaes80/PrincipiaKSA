PrincipiaKSA: Principia's n-body gravity for Kitten Space Agency
(Linux and Windows x64, Apple Silicon; the version is in VERSION)
https://github.com/parsaes80/PrincipiaKSA

Requires StarMap (KSA's mod loader), 0.4.8 or later, and a CPU with AVX and
FMA (Intel Haswell / AMD Zen or later) or Apple Silicon; on others the mod
disables itself.

Install:
  1. Copy this PrincipiaKSA folder to the mods folder:
       Linux:   ~/Documents/My Games/Kitten Space Agency/mods/
       Windows: %USERPROFILE%\Documents\My Games\Kitten Space Agency\mods\
     With the Borea launcher, the mods folder of its game profile instead.
  2. Add to manifest.toml in the folder above the mods folder:
       [[mods]]
       id = "PrincipiaKSA"
       enabled = true
  3. Start KSA through StarMap.

The PrincipiaKSA window shows the n-body prediction, the flight plan (KSA's
planned burns) and the plotting frame. Log: PrincipiaKSA.log here.

PrincipiaKSA is under the MIT licence (licenses/PrincipiaKSA.txt).
Principia is by the Principia authors (MIT licence). The native libraries
also contain abseil, protobuf, re2, gipfeli, zfp, Boost, CORE-MATH and LLVM's
libc++ (and, for Windows, the MinGW-w64 runtime; for Apple Silicon, SIMDe):
their licences are in licenses/.
