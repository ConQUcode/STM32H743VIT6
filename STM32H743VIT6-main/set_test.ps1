
$content = Get-Content '$PSScriptRoot\APPLICATION\Test.c' -Raw
$new_content = $content -replace 'CURRENT_LOOP \| SPEED_LOOP', 'SPEED_LOOP' 
Set-Content '$PSScriptRoot\APPLICATION\Test.c' -Value $new_content

