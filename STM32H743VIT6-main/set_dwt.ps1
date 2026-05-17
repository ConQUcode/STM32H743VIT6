
$content = Get-Content '$PSScriptRoot\BSP\bsp_dwt.c' -Raw
$new_content = $content -replace 'CoreDebug->DEMCR \|= CoreDebug_DEMCR_TRCENA_Msk;', "CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;"
Set-Content '$PSScriptRoot\BSP\bsp_dwt.c' -Value $new_content

