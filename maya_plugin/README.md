## VSCode Project properties to be set - VSCode 2019

TangoCurveViewer
	1. Set as startup project
	2. Configuration properties -> General -> Configuration Type = .dll
	3. Configuration properties -> Advanced -> Target File Extension = .mll
	4. C++ -> General -> Additional Include Directories
		[MayaPath]/include
		[MosekPath]/h
	5. C++ -> Preprocessor -> Preprocessor Definitions
		Add NT_PLUGIN
	6. C++ -> Code Generation -> Runtime Library = Debug DLL(/MDd)
	7. Linker -> General -> Output File = $(OutputDir)$(ProjectName).mll
	8. Linker -> General -> Additional Library Directories
		[MayaPath]\lib
		[MayaPath]\lib2
		[MayaPath]\lib3
	9. Linker -> Input -> Additional Dependencies
		<different options>
		[MosekPath]\bin\mosek64_9_3.lib
		Foundation.lib;
		OpenMaya.lib;
		OpenMayaUI.lib;
		OpenMayaAnim.lib;
		OpenMayaFX.lib;
		OpenMayaRender.lib;
		Image.lib;
		opengl32.lib
	
Curve
	1. Static Library
	2. Additional dependency
		[MosekPath]\h
	
CurvePlugin
	1. Static Library
	2. Additional Dependencies
		[MosekPath]\h