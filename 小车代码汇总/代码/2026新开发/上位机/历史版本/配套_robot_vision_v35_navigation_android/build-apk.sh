#!/bin/sh
# Build a native APK using only the official Android SDK + JDK 17; no Gradle downloads.
set -eu
HERE=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
SDK=${ANDROID_SDK_ROOT:-${ANDROID_HOME:-}}
TOOLS=${BUILD_TOOLS:-${SDK:+$SDK/build-tools/35.0.0}}
ANDROID_JAR=${ANDROID_JAR:-${SDK:+$SDK/platforms/android-35/android.jar}}
if [ -z "$TOOLS" ] || [ ! -x "$TOOLS/aapt2" ] || [ ! -r "$ANDROID_JAR" ]; then
    echo 'Install official Android SDK platform android-35 and build-tools 35.0.0, then set ANDROID_SDK_ROOT.' >&2
    echo 'Or set BUILD_TOOLS=/path/to/build-tools and ANDROID_JAR=/path/to/android.jar.' >&2
    exit 1
fi
BUILD="$HERE/build"
OUT="$HERE/dist"
KEYSTORE=${SIGNING_KEYSTORE:-${DEBUG_KEYSTORE:-$HERE/.local/debug.keystore}}
mkdir -p "$BUILD/classes" "$BUILD/generated" "$BUILD/dex" "$OUT" "$(dirname -- "$KEYSTORE")"
find "$BUILD/classes" "$BUILD/generated" "$BUILD/dex" -type f -delete
"$TOOLS/aapt2" compile --dir "$HERE/res" -o "$BUILD/resources.zip"
"$TOOLS/aapt2" link -o "$BUILD/base.apk" -I "$ANDROID_JAR" --manifest "$HERE/AndroidManifest.xml" --java "$BUILD/generated" --min-sdk-version 26 --target-sdk-version 35 "$BUILD/resources.zip"
find "$HERE/src" "$BUILD/generated" -name '*.java' > "$BUILD/java-sources.txt"
javac -encoding UTF-8 --release 8 -classpath "$ANDROID_JAR" -d "$BUILD/classes" @"$BUILD/java-sources.txt"
find "$BUILD/classes" -name '*.class' > "$BUILD/dex-inputs.txt"
"$TOOLS/d8" --lib "$ANDROID_JAR" --min-api 26 --output "$BUILD/dex" @"$BUILD/dex-inputs.txt"
cp "$BUILD/base.apk" "$BUILD/unsigned.apk"
(cd "$BUILD/dex" && zip -q -j "$BUILD/unsigned.apk" classes.dex)
"$TOOLS/zipalign" -f -p 4 "$BUILD/unsigned.apk" "$BUILD/aligned.apk"
if [ ! -f "$KEYSTORE" ]; then
    keytool -genkeypair -noprompt -keystore "$KEYSTORE" -storepass android -keypass android -alias androiddebugkey -keyalg RSA -keysize 2048 -sigalg SHA256withRSA -validity 10000 -dname 'CN=ACG138 Development,O=Local Robot Project,C=CN'
fi
APK="$OUT/ACG138-Robot-Control-debug.apk"
"$TOOLS/apksigner" sign --ks "$KEYSTORE" --ks-pass pass:android --key-pass pass:android --ks-key-alias androiddebugkey --out "$APK" "$BUILD/aligned.apk"
"$TOOLS/apksigner" verify --verbose --print-certs "$APK" > "$OUT/apk-signature.txt"
"$TOOLS/zipalign" -c -p 4 "$APK"
"$TOOLS/aapt2" dump badging "$APK" > "$OUT/apk-manifest.txt"
(cd "$OUT" && sha256sum ACG138-Robot-Control-debug.apk > SHA256SUMS)
echo "APK built and verified: $APK"
