AppDomain = dotNetClass "System.AppDomain"
Assembly = dotNetClass "System.Reflection.Assembly"
AssemblyName = dotNetClass "System.Reflection.AssemblyName"
exporter_path = @"C:\Users\JLatteux\Documents\GitHub\ROD_Exporter\ROD_Exporter\bin\Debug\ROD_Exporter.dll"
assemName = AssemblyName.GetAssemblyName(exporter_path)
assem = Assembly.Load assemName
exporter = assem.CreateInstance "ROD_Exporter.ROD_ExportG"
frames = #(0,30,60,90,120)
skeleton_filename = @"C:\temp\skeleton.skl"
clip_filename = @"C:\temp\anim.clp"
exporter.Bone_To frames clip_filename skeleton_filename