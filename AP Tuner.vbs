' AP Tuner launcher: starts the app in the background and opens it as its own window (Edge app mode).
' Double-click this file, or use the shortcut that "Install AP Tuner.cmd" creates.
Option Explicit
Dim sh, fso, dir, node, edge, url, cmd
Set sh = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
dir = fso.GetParentFolderName(WScript.ScriptFullName)
url = "http://127.0.0.1:8090"
node = dir & "\node\node.exe"
If Not fso.FileExists(node) Then node = "node"
' start the server hidden (it exits by itself when the app window is closed)
sh.Run """" & node & """ """ & dir & "\src\server.js"" --app", 0, False
WScript.Sleep 1500
edge = ""
If fso.FileExists("C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe") Then edge = "C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
If fso.FileExists("C:\Program Files\Microsoft\Edge\Application\msedge.exe") Then edge = "C:\Program Files\Microsoft\Edge\Application\msedge.exe"
If edge <> "" Then
  sh.Run """" & edge & """ --app=" & url & " --window-size=1400,920 --user-data-dir=""" & sh.ExpandEnvironmentStrings("%LOCALAPPDATA%") & "\AP Tuner\edge-profile""", 1, False
Else
  sh.Run url, 1, False
End If
