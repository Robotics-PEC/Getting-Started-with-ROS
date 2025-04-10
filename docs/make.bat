@ECHO OFF
pushd %~dp0

REM Command file for Sphinx documentation on Windows

REM Set defaults
if "%SPHINXBUILD%" == "" (
    set SPHINXBUILD=.venv\Scripts\python.exe -m sphinx
)
if "%SPHINXAUTOBUILD%" == "" (
    set SPHINXAUTOBUILD=.venv\Scripts\python.exe -m sphinx_autobuild
)
set SOURCEDIR=source
set BUILDDIR=build

REM If no argument passed, show help
if "%1" == "" goto help

REM Setup environment (venv + pip install)
if "%1" == "setup" (
    if not exist ".venv\Scripts\activate.bat" (
        echo Creating virtual environment...
        python -m venv .venv
    ) else (
        echo Virtual environment already exists. Skipping creation.
    )

    echo Installing Python packages...
    .venv\Scripts\python.exe -m pip install -r requirements.txt

    echo.
    echo Setup complete.
    goto end
)

REM Clean build directory
if "%1" == "clean" (
    echo Removing build directory...
    rmdir /S /Q "%BUILDDIR%" 2>nul
    echo Build directory cleaned.
    goto end
)

REM Full clean: build + venv
if "%1" == "fullclean" (
    echo Removing build and virtual environment directories...
    rmdir /S /Q "%BUILDDIR%" 2>nul
    rmdir /S /Q ".venv" 2>nul
    echo Full clean complete.
    goto end
)

REM Ensure virtual environment exists
if not exist ".venv\Scripts\activate.bat" (
    echo.
    echo ERROR: Virtual environment not found. Run `make.bat setup` first.
    echo.
    exit /b 1
)

REM Handle 'dev' target with sphinx-autobuild
if "%1" == "dev" (
    echo Starting sphinx-autobuild for live documentation preview...
    %SPHINXAUTOBUILD% %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%
    goto end
)

REM Handle all other targets with sphinx-build -M
%SPHINXBUILD% -M %1 %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%
goto end

:help
echo.
echo Usage:
echo   make.bat setup       Create virtual environment and install dependencies
echo   make.bat html        Build the documentation
echo   make.bat dev         Start sphinx-autobuild for live preview
echo   make.bat clean       Delete build directory
echo   make.bat fullclean   Delete build directory and virtual environment
echo   make.bat help        Show this help
echo.
%SPHINXBUILD% -M help %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%

:end
popd
