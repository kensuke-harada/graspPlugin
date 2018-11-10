#!/usr/bin/env python

import sys
import re

def scalingPrmFile(target_dir,scale_x,scale_y,scale_z,suffix):
	input_file = None
	output_file= None
	input = target_dir + "/data/prehension.prm"
	output = target_dir + "/data/scaled_prehension" + suffix + ".prm"
	try:
		input_file = open(input,'r')
		output_file = open(output,'w')
		for line in input_file:
			if re.search('GRC(des|min|max)_(Position|Edges)',line):
				word = line.strip().split(" ")
				while word.count("") > 0:
					word.remove("")
				word[1] = str(float(word[1]) * scale_x)
				word[2] = str(float(word[2]) * scale_y)
				word[3] = str(float(word[3]) * scale_z)
				output_file.write(" ".join(word) + '\n')
			else:
				output_file.write(line)
	finally:
		input_file.close()
		output_file.close()

def scalingTrobotwrl(target_dir,scale_x,scale_y,scale_z,suffix):
	input_file = None
	output_file = None
	input = target_dir + "/main.wrl"
	output = target_dir + "/scaled_main" + suffix + ".wrl"
	in_finger = False
	in_hand = False
	try:
		input_file = open(input,'r')
		output_file = open(output,'w')
		for line in input_file:
			if re.search('DEF HAND_[1-3]_L3 Joint {',line):
				in_finger = True
			if re.search('DEF J7 Joint {',line):
				in_hand = True
			if re.search('end of joint HAND_[1-3]_L3',line):
				in_finger = False
			if re.search('end of joint J7',line):
				in_hand = False
			if in_finger and re.search('translation',line):
				word = line.strip().split(" ")
				while word.count("") > 0:
					word.remove("")
				x = float(word[1]) * scale_x
				y = float(word[2]) * scale_y
				z = float(word[3]) * scale_z
				output_file.write(re.sub('translation [-0-9\.]+ [-0-9\.]+ [-0-9\.]+','translation ' + str(x) + ' ' + str(y) + ' ' + str(z),line))
			elif in_hand and re.search('scale',line):
				output_file.write(re.sub('scale [0-9\.]+ [0-9\.]+ [0-9\.]+','scale ' + str(scale_x) + ' ' + str(scale_y) + ' ' + str(scale_z),line))
			else:
				output_file.write(line)
	finally:
		input_file.close()
		output_file.close()

def scalingTrobotyaml(target_dir,scale_x,scale_y,scale_z,suffix):
	intput_file = None
	output_file = None
	input = target_dir + "/TRobot.yaml"
	output = target_dir + "/scaled_TRobot" + suffix + ".yaml"
	try:
		input_file = open(input,'r')
		output_file = open(output,'w')
		for line in input_file:
			if re.search('modelFile:', line):
				output_file.write(re.sub('modelFile: \S+','modelFile: scaled_main' + suffix + '.wrl',line))
			elif re.search('prehensionList:', line):
				output_file.write(re.sub('prehensionList: \S+','prehensionList: [scaled_prehension' + suffix +  ']',line))
			elif re.search('fmax:', line):
				word = line.strip().split(" ")
				while word.count("") > 0:
					word.remove("")
				fmax = float(word[1]) * scale_x * scale_y * scale_z
				output_file.write(re.sub('fmax: \S+','fmax: ' + str(fmax),line))
			else:
				output_file.write(line)
	finally:
		input_file.close()
		output_file.close()

def cpPosFile(target_dir,suffix):
        input = target_dir + "/data/prehension.pos"
	output = target_dir + "/data/scaled_prehension" + suffix + ".pos"
        input_file = None
        output_file = None
	try:
		input_file = open(input,'r')
		output_file = open(output,'w')
                for line in input_file:
                        output_file.write(line)
        finally:
                input_file.close()
                output_file.close()


def generateScaledTrobot(target_dir,scale_x,scale_y,scale_z,suffix=""):
	scalingTrobotwrl(target_dir,scale_x,scale_y,scale_z,suffix)
	scalingTrobotyaml(target_dir,scale_x,scale_y,scale_z,suffix)
	scalingPrmFile(target_dir,scale_x,scale_y,scale_z,suffix)
	cpPosFile(target_dir,suffix)

def main():
	argv = sys.argv
	if len(argv) < 5:
		print 'Usage: python %s scale_x scale_y scale_z suffix' % argv[0]
		quit()
	generateScaledTrobot("../../hrgPlugin/RobotModels/Trobot",float(argv[1]),float(argv[2]),float(argv[3]),argv[4])

if __name__ == '__main__':
	main()

