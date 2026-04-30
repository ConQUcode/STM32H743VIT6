param(
    [string]$ProjectFile = "MDK-ARM/VIT6.uvprojx"
)

$ErrorActionPreference = "Stop"

$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$resolvedProjectInput = $ProjectFile
if (-not [System.IO.Path]::IsPathRooted($resolvedProjectInput)) {
    $resolvedProjectInput = Join-Path $scriptRoot $resolvedProjectInput
}

if (-not (Test-Path $resolvedProjectInput)) {
    throw "Project file not found: $resolvedProjectInput"
}

$portablePathCm4 = "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F"
$portablePathRvdsCm7 = "Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM7/r0p1"
$portablePathGccCm7 = "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1"

function Ensure-FreeRtosGccCm7Port {
    param(
        [string]$Destination,
        [string]$RelativePath
    )

    $requiredFiles = @("port.c", "portmacro.h")
    $missingFiles = @($requiredFiles | Where-Object { -not (Test-Path (Join-Path $Destination $_)) })
    if ($missingFiles.Count -eq 0) {
        return
    }

    $sourceRoots = @()
    if ($env:STM32CUBE_FW_H7_PATH) {
        $sourceRoots += $env:STM32CUBE_FW_H7_PATH
    }
    if ($env:USERPROFILE) {
        $sourceRoots += (Join-Path $env:USERPROFILE "STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1")
    }

    foreach ($sourceRoot in $sourceRoots) {
        $sourcePath = Join-Path $sourceRoot $RelativePath
        $sourceHasAllFiles = $true
        foreach ($requiredFile in $requiredFiles) {
            if (-not (Test-Path (Join-Path $sourcePath $requiredFile))) {
                $sourceHasAllFiles = $false
                break
            }
        }

        if ($sourceHasAllFiles) {
            New-Item -ItemType Directory -Force -Path $Destination | Out-Null
            foreach ($requiredFile in $requiredFiles) {
                Copy-Item -Force -Path (Join-Path $sourcePath $requiredFile) -Destination (Join-Path $Destination $requiredFile)
            }
            Write-Host "Restored FreeRTOS GCC ARM_CM7 port files from $sourcePath"
            return
        }
    }

    throw "FreeRTOS GCC ARM_CM7 port files are missing. Install STM32Cube_FW_H7_V1.12.1 in the default STM32Cube repository, or set STM32CUBE_FW_H7_PATH to the firmware package root."
}
$uvprojxLegacyPortPattern = '(?s)\s*<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*portable/RVDS/ARM_CM4F/port\.c</FilePath>.*?</File>'
$uvoptxLegacyPortPattern = '(?s)\s*<File>\s*<GroupNumber>\d+</GroupNumber>\s*<FileNumber>\d+</FileNumber>\s*<FileType>1</FileType>.*?<PathWithFileName>[^<]*portable/RVDS/ARM_CM4F/port\.c</PathWithFileName>.*?</File>'
$uvprojxAnyPortPattern = '(?s)\s*<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*portable/.+?/port\.c</FilePath>.*?</File>'
$uvoptxAnyPortPattern = '(?s)\s*<File>\s*<GroupNumber>\d+</GroupNumber>\s*<FileNumber>\d+</FileNumber>\s*<FileType>1</FileType>.*?<PathWithFileName>[^<]*portable/.+?/port\.c</PathWithFileName>.*?</File>'
$uvprojxDuplicateGccPattern = '(?s)(\s*<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*portable/GCC/ARM_CM7/r0p1/port\.c</FilePath>.*?</File>)\s*(<File>\s*<FileName>port\.c</FileName>\s*<FileType>1</FileType>\s*<FilePath>[^<]*portable/GCC/ARM_CM7/r0p1/port\.c</FilePath>.*?</File>)'
$uvoptxDuplicateGccPattern = '(?s)(\s*<File>\s*<GroupNumber>\d+</GroupNumber>\s*<FileNumber>\d+</FileNumber>\s*<FileType>1</FileType>.*?<PathWithFileName>[^<]*portable/GCC/ARM_CM7/r0p1/port\.c</PathWithFileName>.*?</File>)\s*(<File>\s*<GroupNumber>\d+</GroupNumber>\s*<FileNumber>\d+</FileNumber>\s*<FileType>1</FileType>.*?<PathWithFileName>[^<]*portable/GCC/ARM_CM7/r0p1/port\.c</PathWithFileName>.*?</File>)'
$projectPath = Resolve-Path $resolvedProjectInput
$projectDir = Split-Path -Parent $projectPath
$projectRoot = Split-Path -Parent $projectDir
$projectName = [System.IO.Path]::GetFileNameWithoutExtension($projectPath)
$filesToPatch = @($projectPath)
$uvoptxPath = Join-Path $projectDir ($projectName + ".uvoptx")

Ensure-FreeRtosGccCm7Port -Destination (Join-Path $projectRoot $portablePathGccCm7) -RelativePath $portablePathGccCm7

if (Test-Path $uvoptxPath) {
    $filesToPatch += (Resolve-Path $uvoptxPath)
}

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)

foreach ($file in $filesToPatch) {
    $content = Get-Content -Path $file -Raw
    $updated = $content

    if ($file -like "*.uvprojx") {
        $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $uvprojxLegacyPortPattern, "", [System.Text.RegularExpressions.RegexOptions]::Singleline)
        $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $uvprojxDuplicateGccPattern, '$1', [System.Text.RegularExpressions.RegexOptions]::Singleline)
    }
    if ($file -like "*.uvoptx") {
        $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $uvoptxLegacyPortPattern, "", [System.Text.RegularExpressions.RegexOptions]::Singleline)
        $updated = [System.Text.RegularExpressions.Regex]::Replace($updated, $uvoptxDuplicateGccPattern, '$1', [System.Text.RegularExpressions.RegexOptions]::Singleline)
    }

    $updated = $updated.Replace($portablePathCm4, $portablePathGccCm7)
    $updated = $updated.Replace($portablePathRvdsCm7, $portablePathGccCm7)

    $duplicatePattern = $null
    if ($file -like "*.uvprojx") {
        $duplicatePattern = $uvprojxAnyPortPattern
    } elseif ($file -like "*.uvoptx") {
        $duplicatePattern = $uvoptxAnyPortPattern
        $matches = [System.Text.RegularExpressions.Regex]::Matches($updated, $duplicatePattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if ($matches.Count -gt 1) {
            for ($i = $matches.Count - 1; $i -ge 1; $i--) {
                $updated = $updated.Remove($matches[$i].Index, $matches[$i].Length)
            }
        }
    }

    if ($updated -eq $content) {
        Write-Host "No FreeRTOS portable path update was needed in $file"
    } else {
        [System.IO.File]::WriteAllText($file, $updated, $utf8NoBom)
        Write-Host "Updated FreeRTOS portable path in $file"
    }

    $verify = Get-Content -Path $file -Raw
    if ($verify.Contains($portablePathCm4)) {
        throw "Patch verification failed: CM4F FreeRTOS portable path is still present in $file"
    }
    if ($verify.Contains("portable/RVDS/ARM_CM4F/port.c")) {
        throw "Patch verification failed: legacy RVDS CM4F port.c entry is still present in $file"
    }
    if ($verify.Contains("Middlewares/Third_Party/FreeRTOS/Source/portable/") -and
        -not $verify.Contains($portablePathGccCm7)) {
        throw "Patch verification failed: GCC ARM_CM7 r0p1 portable path not found in $file"
    }
    foreach ($requiredPortFile in @("port.c", "portmacro.h")) {
        $requiredPortPath = Join-Path (Join-Path $projectRoot $portablePathGccCm7) $requiredPortFile
        if (-not (Test-Path $requiredPortPath)) {
            throw "Patch verification failed: missing $requiredPortPath"
        }
    }
    if ($null -ne $duplicatePattern) {
        $remainingMatches = [System.Text.RegularExpressions.Regex]::Matches($verify, $duplicatePattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if ($remainingMatches.Count -gt 1) {
            throw "Patch verification failed: duplicate FreeRTOS port.c entries are still present in $file"
        }
    }
}

Write-Host "Verification OK: using $portablePathGccCm7"
