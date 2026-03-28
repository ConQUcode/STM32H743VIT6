
$content = Get-Content 'c:\Users\11737\Desktop\VIT6\BSP\bsp_dwt.c' -Raw
$new_content = $content -replace 'CoreDebug->DEMCR \|= CoreDebug_DEMCR_TRCENA_Msk;', "CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55;"
Set-Content 'c:\Users\11737\Desktop\VIT6\BSP\bsp_dwt.c' -Value $new_content

