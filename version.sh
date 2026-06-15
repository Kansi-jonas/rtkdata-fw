#!/bin/bash

# Pfade
BUILD_DIR="./build"
DESKTOP_DIR=~/Desktop
ZIP_FILE="$DESKTOP_DIR/version.zip"

# Dateien, die hinzugefügt werden sollen
FILES=(
    "$BUILD_DIR/bootloader/bootloader.bin"
    "$BUILD_DIR/rtkdata-fw.bin"
    "$BUILD_DIR/partition_table/partition-table.bin"
    "$BUILD_DIR/www.bin"
)

# Prüfen, ob alle Dateien existieren
for FILE in "${FILES[@]}"; do
    if [ ! -f "$FILE" ]; then
        echo "Fehler: Datei nicht gefunden: $FILE"
        exit 1
    fi
done

# Alte ZIP-Datei löschen, falls vorhanden
rm -f "$ZIP_FILE"

# ZIP-Datei erstellen
zip -j "$ZIP_FILE" "${FILES[@]}"

echo "ZIP-Datei erfolgreich erstellt: $ZIP_FILE"