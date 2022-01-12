cl /c /Zd /Ox /I (Get-Location) $args
$filename = [System.IO.Path]::GetFileNameWithoutExtension($args)
dumpbin /LINENUMBERS /disasm:NOBYTES .\$filename.obj | Out-File -FilePath .\$filename.asm
py -3 asm_cleaner.py --i .\$filename.asm --cpp .\$args