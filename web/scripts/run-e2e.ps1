$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..\..")
$webRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$backendOut = Join-Path $repoRoot "runs\phase7a_backend.out.log"
$backendErr = Join-Path $repoRoot "runs\phase7a_backend.err.log"
$frontendOut = Join-Path $repoRoot "runs\phase7a_frontend.out.log"
$frontendErr = Join-Path $repoRoot "runs\phase7a_frontend.err.log"

$backend = Start-Process `
  -FilePath "E:\anaconda\python.exe" `
  -ArgumentList @("-m", "uvicorn", "uav_search.server.app:app", "--host", "127.0.0.1", "--port", "8000") `
  -WorkingDirectory $repoRoot `
  -WindowStyle Hidden `
  -PassThru `
  -RedirectStandardOutput $backendOut `
  -RedirectStandardError $backendErr

$frontend = Start-Process `
  -FilePath "npm.cmd" `
  -ArgumentList @("run", "dev", "--", "--port", "5173") `
  -WorkingDirectory $webRoot `
  -WindowStyle Hidden `
  -PassThru `
  -RedirectStandardOutput $frontendOut `
  -RedirectStandardError $frontendErr

try {
  $backendReady = $false
  $frontendReady = $false
  for ($i = 0; $i -lt 90; $i++) {
    if (-not $backendReady) {
      try {
        $null = Invoke-RestMethod http://127.0.0.1:8000/api/health -TimeoutSec 2
        $backendReady = $true
      } catch {}
    }
    if (-not $frontendReady) {
      try {
        $response = Invoke-WebRequest http://127.0.0.1:5173 -UseBasicParsing -TimeoutSec 2
        if ($response.StatusCode -eq 200) {
          $frontendReady = $true
        }
      } catch {}
    }
    if ($backendReady -and $frontendReady) {
      break
    }
    Start-Sleep -Seconds 1
  }

  if (-not ($backendReady -and $frontendReady)) {
    Write-Output "backendReady=$backendReady frontendReady=$frontendReady"
    exit 1
  }

  Push-Location $webRoot
  try {
    npx playwright test --config playwright.manual.config.ts
    if ($LASTEXITCODE -ne 0) {
      exit $LASTEXITCODE
    }
  } finally {
    Pop-Location
  }
} finally {
  if ($frontend -and -not $frontend.HasExited) {
    Stop-Process -Id $frontend.Id -Force
  }
  if ($backend -and -not $backend.HasExited) {
    Stop-Process -Id $backend.Id -Force
  }
  Get-CimInstance Win32_Process |
    Where-Object { $_.CommandLine -like "*zhencha\web*vite*" -or $_.CommandLine -like "*npm-cli.js*run dev*--port 5173*" } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
}
