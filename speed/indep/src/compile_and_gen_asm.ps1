$workingDir = Get-Location
.\speed\indep\src\devenv_setup.ps1

$filename = [System.IO.Path]::GetFileNameWithoutExtension($args)
$srcDir = "${workingDir}\speed\indep\src"

cl /Fo"${srcDir}\${filename}.obj" /c /Zd /Ox /I "${workingDir}" /I "${srcDir}" $args
dumpbin /LINENUMBERS /disasm:NOBYTES "${srcDir}\${filename}.obj" | Out-File -FilePath "${srcDir}\${filename}.asm"
py -3 .\speed\indep\src\asm_cleaner.py --i "${srcDir}\$filename.asm" --cpp "${args}"