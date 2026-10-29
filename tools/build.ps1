# Builds the firmware in a Podman container (see tools/Containerfile.avr).
# Usage, from anywhere:  tools\build.ps1 [make targets...]   e.g. tools\build.ps1 clean all
$ErrorActionPreference = "Stop"
$repo = Split-Path $PSScriptRoot -Parent
$image = "adapt-ffb-joy-avr"

podman image exists $image 2>$null
if ($LASTEXITCODE -ne 0) {
    podman build -t $image -f (Join-Path $PSScriptRoot "Containerfile.avr") $repo
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

# One make per target: the makefile creates .dep/ while parsing, so "make clean all" in one call fails
$targets = if ($args.Count) { $args } else { @("all") }
foreach ($target in $targets) {
    podman run --rm -v "${repo}:/wt" $image make $target
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
