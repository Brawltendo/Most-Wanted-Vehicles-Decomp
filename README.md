# Most Wanted Vehicles Decomp
A WIP matching decompilation of Need for Speed Most Wanted's vehicle physics. This is NOT fully usable or pretty code in its current state, nor does it compile to a full program; you'll find a lot of instances where code was only written a certain way so that it would compile to matching assembly (stuff like abusing `volatile`, assigning temp variables solely for proper regalloc, just oddly written code in general, etc...). This is all purely for research purposes. Original code belongs to Electronic Arts.

# Decompilation requirements
If you want to decompile and match functions for yourself, you'll need to install a copy of Visual Studio .NET 2003, since Most Wanted was compiled using MSVC 7.1. If you've installed VS2003 somewhere other than the default location, just change the path inside `devenv_setup.ps1` to your install location. If you don't have Python 3 installed, you'll need to do that because everything after this point requires it. Then once you're ready to compile, run `compile_and_gen_asm.ps1` with the relative path to your cpp file within `../speed/indep/src`, or else the compiler will fail if you try to include anything. For example, `.\compile_and_gen_asm.ps1 .\physics\behaviors\chassis.cpp` will compile any functions inside `chassis.cpp` and any functions from included files, and it'll also generate an asm file with the corresponding assembly. The output is also cleaned up for easier diffing.

For the best results when comparing functions, disassemble your game's executable using MSVC's `dumpbin` utility with this PowerShell command: `dumpbin /disasm:NOBYTES speed.exe | Out-File -FilePath speed.asm`. Then when you find a function that you want to match, run `asm_splitter.py`. Pass your assembly file, function start and end addresses, and the name of the output file as arguments for the script (ie:`py -3 asm_splitter.py --i speed.asm --addr 0068D6A0 0068D7F4 --name chassis`), and the result will be a cleaned up asm file that will make matching using a differ much easier.

# VSCode Task setup
If you're using VSCode and want to make the compilation process much easier, you can set up your `tasks.json` like the example below. This will allow you to compile the currently active cpp file and generate an asm file using the default build task hotkey (Ctrl+Shift+B by default)
```
{
	// See https://go.microsoft.com/fwlink/?LinkId=733558
	// for the documentation about the tasks.json format
	"version": "2.0.0",
	"windows": {
        "options": {
            "shell": {
                "executable": "powershell.exe",
                "args": [
                    "-NoProfile",
                    "-ExecutionPolicy",
                    "Bypass",
                    "-Command"
                ]
            }
        }
    },
	"tasks": [
		{
			"label": "compile_and_gen_asm",
			"type": "shell",
			"command": "${workspaceFolder}\\speed\\indep\\src\\compile_and_gen_asm.ps1",
			"args": [
				"${file}"
			],
			"problemMatcher": [
				"$msCompile"
			],
			"group": {
				"kind": "build",
				"isDefault": true
			},
			"detail": "Compiles current file with cl.exe, then generates and formats asm"
		}
	]
}
```