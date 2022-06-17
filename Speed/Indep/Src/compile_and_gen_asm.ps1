$workingDir = Get-Location
.\speed\indep\src\devenv_setup.ps1

$filename = [System.IO.Path]::GetFileNameWithoutExtension($args)
$srcDir = "${workingDir}\speed\indep\src"

cl /Fo"${srcDir}\${filename}.obj" /c /EHsc /Zd /Ox /I "${workingDir}" /I "${srcDir}" /I "${workingDir}\Speed\Win32\Libs\STL\STLport-4.5\stlport" $args
dumpbin /LINENUMBERS /disasm:NOBYTES "${srcDir}\${filename}.obj" | Out-File -FilePath "${srcDir}\${filename}.asm"
py -3 .\speed\indep\src\asm_cleaner.py --i "${srcDir}\$filename.asm" --cpp "${args}"