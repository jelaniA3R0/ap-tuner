# Renders the before/after comparison video frames (MJPEG bundle) from a scene JSON with GDI+.
# Usage: powershell -File render-frames.ps1 scene.json frames.bin
param([string]$SceneFile, [string]$OutFile)
$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$scene = Get-Content -Raw -Encoding UTF8 $SceneFile | ConvertFrom-Json
$W = 1280; $H = 720; $fps = [int]$scene.fps; $dur = [double]$scene.duration
$nFrames = [int][Math]::Floor($dur * $fps)
$bg = [System.Drawing.Color]::FromArgb(11, 17, 32)
$panelBg = [System.Drawing.Color]::FromArgb(18, 26, 43)
$grid = [System.Drawing.Color]::FromArgb(34, 48, 73)
$text = [System.Drawing.Color]::FromArgb(230, 237, 247)
$muted = [System.Drawing.Color]::FromArgb(147, 160, 180)
$cBefore = [System.Drawing.Color]::FromArgb(249, 115, 22)
$cAfter = [System.Drawing.Color]::FromArgb(56, 189, 248)
$cTarget = [System.Drawing.Color]::FromArgb(250, 204, 21)
$fontBig = New-Object System.Drawing.Font('Segoe UI', 20, [System.Drawing.FontStyle]::Bold)
$fontMid = New-Object System.Drawing.Font('Segoe UI', 13, [System.Drawing.FontStyle]::Bold)
$font = New-Object System.Drawing.Font('Segoe UI', 10.5)
$fontSmall = New-Object System.Drawing.Font('Segoe UI', 9)
$jpgCodec = [System.Drawing.Imaging.ImageCodecInfo]::GetImageEncoders() | Where-Object { $_.MimeType -eq 'image/jpeg' }
$encParams = New-Object System.Drawing.Imaging.EncoderParameters(1)
$encParams.Param[0] = New-Object System.Drawing.Imaging.EncoderParameter([System.Drawing.Imaging.Encoder]::Quality, [long]82)

function Interp($t, $ts, $vs) {
  # linear interpolation on an ascending time array (arrays from JSON)
  $n = $ts.Count; if ($n -eq 0) { return 0.0 }
  if ($t -le $ts[0]) { return [double]$vs[0] }
  if ($t -ge $ts[$n - 1]) { return [double]$vs[$n - 1] }
  $lo = 0; $hi = $n - 1
  while ($hi - $lo -gt 1) { $mid = [int](($lo + $hi) / 2); if ($ts[$mid] -le $t) { $lo = $mid } else { $hi = $mid } }
  $f = ([double]$t - [double]$ts[$lo]) / ([double]$ts[$hi] - [double]$ts[$lo] + 1e-9)
  return [double]$vs[$lo] + $f * ([double]$vs[$hi] - [double]$vs[$lo])
}
function Rot($p, $roll, $pitch, $yaw) {
  # body (x fwd, y right, z down) -> world, ZYX Euler, angles in degrees
  $cr = [Math]::Cos($roll * 0.0174533); $sr = [Math]::Sin($roll * 0.0174533)
  $cp = [Math]::Cos($pitch * 0.0174533); $sp = [Math]::Sin($pitch * 0.0174533)
  $cy = [Math]::Cos($yaw * 0.0174533); $sy = [Math]::Sin($yaw * 0.0174533)
  $x = $p[0]; $y = $p[1]; $z = $p[2]
  $X = $cp * $cy * $x + ($sr * $sp * $cy - $cr * $sy) * $y + ($cr * $sp * $cy + $sr * $sy) * $z
  $Y = $cp * $sy * $x + ($sr * $sp * $sy + $cr * $cy) * $y + ($cr * $sp * $sy - $sr * $cy) * $z
  $Z = -$sp * $x + $sr * $cp * $y + $cr * $cp * $z
  return @($X, $Y, $Z)
}
function Proj($P, $cx, $cy, $k) {
  # view from behind and above: screen x = world y, screen y = world z (down) minus a bit of forward x
  return New-Object System.Drawing.PointF(($cx + $k * $P[1]), ($cy + $k * ($P[2] * 0.95 - $P[0] * 0.42)))
}
function DrawVehicle($g, $kind, $roll, $pitch, $yaw, $cx, $cy, $k, $col) {
  $pen = New-Object System.Drawing.Pen($col, 3); $pen.LineJoin = 'Round'
  $thin = New-Object System.Drawing.Pen($col, 1.5)
  $segs = @()
  if ($kind -eq 'quad') {
    $a = 0.28; $r = 0.11
    $arms = @(@($a, $a, 0), @($a, -$a, 0), @(-$a, $a, 0), @(-$a, -$a, 0))
    foreach ($m in $arms) { $segs += ,@(@(0, 0, 0), $m) }
    $segs += ,@(@(0.12, 0, 0), @(0.12, 0, -0.07)); $segs += ,@(@(0.12, 0, -0.07), @(-0.08, 0, -0.07)) # canopy hint
    foreach ($m in $arms) { $pts = @(); for ($i = 0; $i -le 16; $i++) { $th = $i * 0.3927; $pts += ,@(($m[0] + $r * [Math]::Cos($th)), ($m[1] + $r * [Math]::Sin($th)), ($m[2] - 0.02)) }; for ($i = 0; $i -lt 16; $i++) { $segs += ,@($pts[$i], $pts[$i + 1]) } }
    $segs += ,@(@(0, 0, 0), @(0.22, 0, 0)) # nose marker
  } else {
    $segs += ,@(@(-0.7, 0, 0), @(0.8, 0, 0))              # fuselage
    $segs += ,@(@(0.8, 0, 0), @(0.55, 0, -0.1)); $segs += ,@(@(0.55, 0, -0.1), @(0.2, 0, -0.1)); $segs += ,@(@(0.2, 0, -0.1), @(0.1, 0, 0))   # canopy
    $wing = @(@(0.3, -1.1, 0), @(0.3, 1.1, 0), @(-0.1, 1.1, 0), @(-0.1, -1.1, 0))                          # wing outline (chord 0.4 m)
    for ($i = 0; $i -lt 4; $i++) { $segs += ,@($wing[$i], $wing[($i + 1) % 4]) }
    $tail = @(@(-0.5, -0.4, 0), @(-0.5, 0.4, 0), @(-0.7, 0.4, 0), @(-0.7, -0.4, 0))                          # tailplane outline
    for ($i = 0; $i -lt 4; $i++) { $segs += ,@($tail[$i], $tail[($i + 1) % 4]) }
    $segs += ,@(@(-0.5, 0, 0), @(-0.55, 0, -0.3)); $segs += ,@(@(-0.55, 0, -0.3), @(-0.72, 0, -0.3)); $segs += ,@(@(-0.72, 0, -0.3), @(-0.7, 0, 0))   # fin
    $booms = @(@(0.35, -0.5), @(0.35, 0.5), @(-0.35, -0.5), @(-0.35, 0.5))
    foreach ($b in $booms) { $segs += ,@(@(-0.35, ($b[1]), 0), @(0.35, ($b[1]), 0)); $pts = @(); for ($i = 0; $i -le 16; $i++) { $th = $i * 0.3927; $pts += ,@(($b[0] + 0.14 * [Math]::Cos($th)), ($b[1] + 0.14 * [Math]::Sin($th)), -0.03) }; for ($i = 0; $i -lt 16; $i++) { $segs += ,@($pts[$i], $pts[$i + 1]) } }
  }
  foreach ($s in $segs) {
    $p1 = Proj (Rot $s[0] $roll $pitch $yaw) $cx $cy $k
    $p2 = Proj (Rot $s[1] $roll $pitch $yaw) $cx $cy $k
    $g.DrawLine($pen, $p1, $p2)
  }
  $pen.Dispose(); $thin.Dispose()
}
function DrawHorizon($g, $roll, $pitch, $x, $y, $w, $h) {
  $state = $g.Save()
  $g.SetClip((New-Object System.Drawing.Rectangle($x, $y, $w, $h)))
  $cx = $x + $w / 2; $cy = $y + $h / 2 + $pitch * 3.0
  $g.TranslateTransform($cx, $cy); $g.RotateTransform(-$roll); $g.TranslateTransform(-$cx, -$cy)
  $sky = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(20, 38, 64))
  $gnd = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(28, 36, 30))
  $g.FillRectangle($sky, $x - 400, $cy - 900, $w + 800, 900)
  $g.FillRectangle($gnd, $x - 400, $cy, $w + 800, 900)
  $hp = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(120, 140, 170), 1)
  $g.DrawLine($hp, $x - 400, $cy, $x + $w + 400, $cy)
  for ($d = -40; $d -le 40; $d += 10) { if ($d -ne 0) { $yy = $cy - $d * 3.0; $g.DrawLine($hp, $cx - 60, $yy, $cx + 60, $yy) } }
  $g.Restore($state); $sky.Dispose(); $gnd.Dispose(); $hp.Dispose()
}
function DrawTrace($g, $title, $t, $ts, $target, $actual, $x, $y, $w, $h, $col, $span, $ymax) {
  $g.FillRectangle((New-Object System.Drawing.SolidBrush($panelBg)), $x, $y, $w, $h)
  $gp = New-Object System.Drawing.Pen($grid, 1)
  $g.DrawRectangle($gp, $x, $y, $w, $h)
  $g.DrawLine($gp, $x, $y + $h / 2, $x + $w, $y + $h / 2)
  $g.DrawString($title, $fontSmall, (New-Object System.Drawing.SolidBrush($muted)), $x + 4, $y + 2)
  $t0 = $t - $span
  $pt = New-Object System.Drawing.Pen($cTarget, 1.5); $pt.DashPattern = @(3, 3)
  $pa = New-Object System.Drawing.Pen($col, 2)
  $prevT = $null; $prevA = $null
  $n = $ts.Count
  for ($i = 0; $i -lt $n; $i++) {
    $tt = [double]$ts[$i]; if ($tt -lt $t0) { continue }; if ($tt -gt $t) { break }
    $px = $x + ($tt - $t0) / $span * $w
    $pyT = $y + $h / 2 - ([double]$target[$i] / $ymax) * ($h / 2 - 4)
    $pyA = $y + $h / 2 - ([double]$actual[$i] / $ymax) * ($h / 2 - 4)
    if ($pyT -lt $y) { $pyT = $y }; if ($pyT -gt $y + $h) { $pyT = $y + $h }; if ($pyA -lt $y) { $pyA = $y }; if ($pyA -gt $y + $h) { $pyA = $y + $h }
    if ($prevT -ne $null) { $g.DrawLine($pt, [float]$prevT[0], [float]$prevT[1], [float]$px, [float]$pyT); $g.DrawLine($pa, [float]$prevA[0], [float]$prevA[1], [float]$px, [float]$pyA) }
    $prevT = @($px, $pyT); $prevA = @($px, $pyA)
  }
  $pt.Dispose(); $pa.Dispose(); $gp.Dispose()
}
function DrawPanel($g, $data, $label, $col, $x0, $t, $kind) {
  $pw = 620; $ph = 640; $y0 = 70
  $g.FillRectangle((New-Object System.Drawing.SolidBrush($panelBg)), $x0, $y0, $pw, $ph)
  $g.DrawRectangle((New-Object System.Drawing.Pen($grid, 1)), $x0, $y0, $pw, $ph)
  $g.DrawString($label, $fontMid, (New-Object System.Drawing.SolidBrush($col)), $x0 + 12, $y0 + 8)
  $gtxt = "P " + $data.gains.roll.P + "   I " + $data.gains.roll.I + "   D " + $data.gains.roll.D + "   angle P " + $data.gains.roll.angleP + "   FLTD " + $data.gains.roll.FLTD + " Hz"
  $g.DrawString($gtxt, $font, (New-Object System.Drawing.SolidBrush($muted)), $x0 + 12, $y0 + 34)
  # attitude at time t
  $roll = Interp $t $data.att.t $data.att.roll; $pitch = Interp $t $data.att.t $data.att.pitch
  $yaw = (Interp $t $data.att.t $data.att.yaw) - [double]$data.att.yaw[0]
  # 3D view box
  $vx = $x0 + 12; $vy = $y0 + 60; $vw = 400; $vh = 300
  DrawHorizon $g $roll $pitch $vx $vy $vw $vh
  $scale = 230; if ($kind -ne 'quad') { $scale = 115 }
  DrawVehicle $g $kind $roll $pitch $yaw ($vx + $vw / 2) ($vy + $vh / 2 + 10) $scale $col
  $g.DrawRectangle((New-Object System.Drawing.Pen($grid, 1)), $vx, $vy, $vw, $vh)
  $g.DrawString(("roll " + $roll.ToString('0.0') + [char]0x00B0 + "   pitch " + $pitch.ToString('0.0') + [char]0x00B0), $font, (New-Object System.Drawing.SolidBrush($text)), $vx + 6, $vy + $vh - 24)
  # stick box
  $sx = $x0 + 430; $sy = $y0 + 60; $ss = 120
  $g.FillRectangle((New-Object System.Drawing.SolidBrush($bg)), $sx, $sy, $ss, $ss)
  $g.DrawRectangle((New-Object System.Drawing.Pen($grid, 1)), $sx, $sy, $ss, $ss)
  $g.DrawLine((New-Object System.Drawing.Pen($grid, 1)), $sx + $ss / 2, $sy, $sx + $ss / 2, $sy + $ss); $g.DrawLine((New-Object System.Drawing.Pen($grid, 1)), $sx, $sy + $ss / 2, $sx + $ss, $sy + $ss / 2)
  $rr = (Interp $t $data.rc.t $data.rc.roll) - 1500; $rp = (Interp $t $data.rc.t $data.rc.pitch) - 1500
  $dx = $sx + $ss / 2 + $rr / 500 * ($ss / 2 - 6); $dy = $sy + $ss / 2 - $rp / 500 * ($ss / 2 - 6)
  $g.FillEllipse((New-Object System.Drawing.SolidBrush($text)), [float]($dx - 7), [float]($dy - 7), 14, 14)
  $g.DrawString('sticks', $fontSmall, (New-Object System.Drawing.SolidBrush($muted)), $sx + 4, $sy + $ss + 2)
  # metrics box
  $mx = $x0 + 430; $my = $y0 + 200; $mb = New-Object System.Drawing.SolidBrush($text)
  $m = $data.metrics
  $lines = @()
  if ($m.roll) { $lines += ("roll  tracking " + [int]$m.roll.trackingPct + "%   lag " + [int]$m.roll.lagMs + " ms") }
  if ($m.roll.step) { $lines += ("      rise " + [int]$m.roll.step.riseMs + " ms  overshoot " + [int]$m.roll.step.overshootPct + "%") }
  if ($m.roll.angle -and $m.roll.angle.step) { $lines += ("      angle rise " + [int]$m.roll.angle.step.riseMs + " ms  os " + [int]$m.roll.angle.step.overshootPct + "%") }
  if ($m.pitch) { $lines += ("pitch tracking " + [int]$m.pitch.trackingPct + "%   lag " + [int]$m.pitch.lagMs + " ms") }
  if ($m.pitch.step) { $lines += ("      rise " + [int]$m.pitch.step.riseMs + " ms  overshoot " + [int]$m.pitch.step.overshootPct + "%") }
  if ($m.pitch.angle -and $m.pitch.angle.step) { $lines += ("      angle rise " + [int]$m.pitch.angle.step.riseMs + " ms  os " + [int]$m.pitch.angle.step.overshootPct + "%") }
  $ly = $my; foreach ($l in $lines) { $g.DrawString($l, $fontSmall, $mb, $mx, $ly); $ly += 17 }
  # traces
  DrawTrace $g 'roll rate: demanded (dashed) vs actual, deg/s' $t $data.roll.t $data.roll.target $data.roll.actual ($x0 + 12) ($y0 + 372) 596 125 $col 5.0 150
  DrawTrace $g 'pitch rate: demanded (dashed) vs actual, deg/s' $t $data.pitch.t $data.pitch.target $data.pitch.actual ($x0 + 12) ($y0 + 505) 596 125 $col 5.0 150
}

$fs = [IO.File]::Create($OutFile)
$bw = New-Object IO.BinaryWriter($fs)
$bmp = New-Object System.Drawing.Bitmap($W, $H)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = 'AntiAlias'; $g.TextRenderingHint = 'ClearTypeGridFit'
$sw = [Diagnostics.Stopwatch]::StartNew()
for ($f = 0; $f -lt $nFrames; $f++) {
  $t = $f / $fps
  $g.Clear($bg)
  $g.DrawString($scene.title, $fontBig, (New-Object System.Drawing.SolidBrush($text)), 14, 10)
  $g.DrawString(("t = " + $t.ToString('0.0') + " s   same stick sequence, before and after tuning"), $font, (New-Object System.Drawing.SolidBrush($muted)), 14, 44)
  DrawPanel $g $scene.before 'BEFORE: starting gains' $cBefore 14 $t $scene.kind
  DrawPanel $g $scene.after 'AFTER: AP Tuner' $cAfter 646 $t $scene.kind
  $ms = New-Object IO.MemoryStream
  $bmp.Save($ms, $jpgCodec, $encParams)
  $bytes = $ms.ToArray(); $ms.Dispose()
  $bw.Write([int]$bytes.Length); $bw.Write($bytes)
  if ($f % 100 -eq 0) { Write-Host ("frame " + $f + "/" + $nFrames + "  " + [int]$sw.Elapsed.TotalSeconds + " s") }
}
$bw.Flush(); $bw.Dispose(); $fs.Dispose(); $g.Dispose(); $bmp.Dispose()
Write-Host ("done: " + $nFrames + " frames in " + [int]$sw.Elapsed.TotalSeconds + " s")
