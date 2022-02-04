# Splits out and sanitizes a function from an asm file generated by MSVC's dumpbin program.

import argparse

# add parser arguments
parser = argparse.ArgumentParser()
parser.add_argument('--i', type=str, required=True, help=
					'The input assembly file.')
parser.add_argument('--addr', type=str, required=True, nargs=2, help=
					'''The start and end addresses (in hex, unprefixed) of the function to be split. 
					Example: --addr 0044FB90 0044FBAF''')
parser.add_argument('--name', type=str, required=True, help=
					'''The output file name without an extension. 
					The suffix '_orig' will be added so it doesn't conflict with the compiler's asm output.''')
args = parser.parse_args()

# open the file as utf-16 even though it's cp1251
# this ensures that we can actually read lines properly
with open(args.i, 'r', encoding='utf-16') as inAsm:
	foundStartAddr = False
	linesToFix = []
	for line in inAsm:
		if line:
			# find function start
			if line.startswith("  " + args.addr[0] + ':'):
				foundStartAddr = True
				print('Found function start')

			# add subsequent lines
			if foundStartAddr:
				linesToFix.append(line[:-1])
				# end loop if this line was the end of the function
				if line.startswith("  " + args.addr[1] + ':'):
					print('Found function end')
					break

jmpInsts = [
	'jmp', 
	'jo', 
	'jno', 
	'js', 
	'jns', 
	'je', 'jz', 
	'jne', 'jnz', 
	'jb', 'jnae', 'jc', 
	'jnb', 'jae', 'jnc', 
	'jbe', 'jna', 
	'ja', 'jnbe', 
	'jl','jnge', 
	'jge', 'jnl', 
	'jle', 'jng', 
	'jg', 'jnle', 
	'jp', 'jpe', 
	'jnp', 'jpo', 
	'jcxz', 'jecxz'
]

with open(args.name + '_orig.asm', 'w') as outAsm:
	for line in linesToFix:
		origAddr = line[2:10]
		instruction = line[12:24]
		instData = line[24:]
		newAddr = format(int(origAddr, 16) - int(args.addr[0], 16), '08X')

		# build sanitized string
		newLine = []
		newLine.append('  ' + newAddr + ': ' + instruction)
		hasJmpInst = False

		for inst in jmpInsts:
			# strip whitespace and check for equality to avoid duplicate output
			if inst == instruction.strip():
				# check for valid int value
				numIsValid = True
				try:
					jmpAddr = int(instData, 16)
				except:
					numIsValid = False

				# make sure the address is within the function boundaries
				if (jmpAddr >= int(args.addr[0], 16) or jmpAddr <= int(args.addr[1], 16)) and numIsValid:
					newLine.append(format(jmpAddr - int(args.addr[0], 16), '08X'))
					hasJmpInst = True

		if '[' in instData and '+' in instData and ']' in instData:
			newLine.append(FixUpOffsets(instData, '+'))
		elif '[' in instData and '-' in instData and ']' in instData:
			newLine.append(FixUpOffsets(instData, '-'))
		elif not hasJmpInst:
		#if not hasJmpInst:
			newLine.append(instData)

		outAsm.write("".join(newLine) + '\n')

		# Fixes up offsets to display how they do for obj files with dumpbin's output
		# For some reason dumping an exe results in a different output compared to an obj
		def FixUpOffsets(inStr, symbol):
			preBracket = inStr.split('[')[0] + '['
			split = inStr.split('[')[1].split(symbol)
			preSym = split[0] if len(split) < 3 else (split[0] + '+' + split[1])
			# we always want the last element of the list here or else stuff like dword ptr [esp+ebp*4+00000128h] will throw an error
			postSym = inStr.split(']')[0].split(symbol)[-1]
			postBracket = inStr.split(']')[1]

			hasHexSpecifier = False
			offset = 0
			if 'h' in postSym:
				hasHexSpecifier = True
				offset = int(postSym[:-1], 16)
			else:
				offset = int(postSym, 16)

			formatStr = ''
			if offset >= 0xA and offset < 0x10:
				formatStr = '02X'
			elif offset >= 0xA0 and offset < 0x100:
				formatStr = '03X'
			else:
				formatStr = '01X'

			if hasHexSpecifier:
				return preBracket + preSym + symbol + format(offset, formatStr) + 'h' + ']' + postBracket
			else:
				return preBracket + preSym + symbol + format(offset, formatStr) + ']' + postBracket
