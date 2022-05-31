function Invoke-CmdScript {
	param(
	  [String] $scriptName
	)
	$cmdLine = """$scriptName"" $args & set"
	& $env:SystemRoot\system32\cmd.exe /c $cmdLine |
	Select-String '^([^=]*)=(.*)$' | ForEach-Object {
	  $varName = $_.Matches[0].Groups[1].Value
	  $varValue = $_.Matches[0].Groups[2].Value
	  Set-Item Env:$varName $varValue
	}
}

Invoke-CmdScript "C:\Program Files (x86)\Microsoft Visual Studio .NET 2003\Common7\Tools\vsvars32.bat"
write-host "`nVisual Studio 2003 Command Prompt variables set." -ForegroundColor Yellow