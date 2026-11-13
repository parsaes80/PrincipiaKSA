// The CoreFoundation functions abseil's time zones use, without Apple's SDK.
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef long CFIndex;
typedef unsigned char Boolean;
typedef unsigned int CFStringEncoding;
typedef const void* CFTypeRef;
typedef const struct __CFString* CFStringRef;
typedef const struct __CFTimeZone* CFTimeZoneRef;

enum { kCFStringEncodingUTF8 = 0x08000100 };

CFTimeZoneRef CFTimeZoneCopyDefault(void);
CFStringRef CFTimeZoneGetName(CFTimeZoneRef tz);
CFIndex CFStringGetLength(CFStringRef string);
CFIndex CFStringGetMaximumSizeForEncoding(CFIndex length, CFStringEncoding encoding);
Boolean CFStringGetCString(CFStringRef string, char* buffer, CFIndex size, CFStringEncoding encoding);
void CFRelease(CFTypeRef object);

#ifdef __cplusplus
}
#endif
