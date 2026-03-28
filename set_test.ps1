
$content = Get-Content 'c:\Users\11737\Desktop\VIT6\APPLICATION\Test.c' -Raw
$new_content = $content -replace 'CURRENT_LOOP \| SPEED_LOOP', 'SPEED_LOOP' 
Set-Content 'c:\Users\11737\Desktop\VIT6\APPLICATION\Test.c' -Value $new_content

