# This should be in ...\ZED SDK\samples\bin
# You will need to make folders for the files and put the paths on lines 5-7
# line 5 is svo files, line 6 is where the converted files go, line 7 is where files go if they need to be repaired 
import subprocess, os
folderPath = "C:\\Users\\mattl\\Desktop\\zedVids"
outputPath = "C:\\Users\\mattl\\Desktop\\zedConverts"
errorPath = "C:\\Users\\mattl\\Desktop\\zedMechanic"
files = []
for file in os.listdir(folderPath):
	f = os.path.join(folderPath, file)
	if os.path.isfile(f):
		# print(f[:-3] + "avi")
		files.append("\""+ f + "\"")
# print(files)

for pagen in files:
	command = "ZED_SVO_EXPORT.exe "
	# print(pagen)
	x = pagen[:-4] + "avi\""
	x=x.replace(folderPath, outputPath)
	# print(x)
	command += pagen + " " + x + " 0"	 
	print(command)
	out = subprocess.run(command, capture_output=True, shell=True)
	string1 = str(out.stdout)
	if ("Error" in string1):
		print("was error")
		print(pagen)
		move = pagen.replace(folderPath, errorPath)
		moveCMD = "move /Y " + pagen + " " + move
		print(moveCMD)
		subprocess.run(moveCMD, shell=True)
	else:
		print(string1)
	# print(out.stdout)
	