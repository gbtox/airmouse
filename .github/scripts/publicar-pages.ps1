[CmdletBinding()]
param(
  [string]$Message = "Actualizar sitio de GitHub Pages"
)

$ErrorActionPreference = "Stop"
$repositoryRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
Push-Location $repositoryRoot

try {
  $branch = & git branch --show-current
  if ($LASTEXITCODE -ne 0) {
    throw "No se pudo determinar la rama actual."
  }
  if ($branch -ne "main") {
    throw "La publicación requiere la rama main; rama actual: $branch"
  }

  $changes = & git status --porcelain -- index.html .github/scripts/publicar-pages.ps1
  if ($LASTEXITCODE -ne 0) {
    throw "No se pudo revisar el estado de index.html."
  }
  if (-not $changes) {
    Write-Host "No hay cambios del sitio ni del script para publicar."
    return
  }

  & git add -- index.html .github/scripts/publicar-pages.ps1
  if ($LASTEXITCODE -ne 0) {
    throw "No se pudo preparar index.html."
  }

  & git commit --only -m $Message -- index.html .github/scripts/publicar-pages.ps1
  if ($LASTEXITCODE -ne 0) {
    throw "No se pudo crear el commit de index.html."
  }

  & git push origin main
  if ($LASTEXITCODE -ne 0) {
    throw "No se pudo publicar en origin/main."
  }

  Write-Host "Publicado. GitHub Pages: https://gbtox.github.io/airmouse/"
}
finally {
  Pop-Location
}
