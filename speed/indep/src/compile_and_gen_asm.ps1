cl /c /Ogty /I (Get-Location) $args
$filename = [System.IO.Path]::GetFileNameWithoutExtension($args)
dumpbin /disasm:NOBYTES .\$filename.obj | Out-File -FilePath .\$filename.asm