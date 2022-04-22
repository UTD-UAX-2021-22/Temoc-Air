# This file needs to be in ...\ZED SDK\tools
# Make a folder for files to be repaired, add path at line 5 then you can run it with files in the folder
import subprocess, os 
folderPath = "C:\\Users\\mattl\\Desktop\\zedMechanic"
files = []
for file in os.listdir(folderPath):
	f = os.path.join(folderPath, file)
	if os.path.isfile(f):
		print(f[:-3] + "avi")
		files.append("\""+ f + "\"")
# print(files)

for pagen in files:
	command = "\"ZED SVOEditor.exe\" -repair "
	# print(pagen)
	# x = pagen[:-4] + "avi\""
	# x=x.replace(folderPath, outputPath)
	# print(x)
	command += pagen 	 
	print(command)
	subprocess.run(command, shell=True)
	