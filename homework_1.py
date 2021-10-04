import random
import os
from collections import Counter
import math
import heapq
from heapq import heappop, heappush

############################################################################
##                              OSZTÁLYOK                                 ##
############################################################################

class Elemek:
	karakterek=[]			 ## pl: n,x,l,z,...
	mennyiség=[]			## pl: 4,3,2,2,...
	kód=[]					## pl: 1110001,101101,...
	előfordulás=[]			## pl: 0.04, 0.75,... Relatív gyakoriság
	informácio_mennyiség=[] ## pl: 3.321, 3.321, ...
	entrópia=0

############################################################################
##                               VÁLTOZÓK                                 ##
############################################################################

üzenet=[]
előfordulás=[]
rnd=[]
infó_menny=[]
doksi=open("értékek.txt",'w+')
singular_key=13
singular=[]

############################################################################
##                              FÜGGVÉNYEK                                ##
############################################################################

def print_object():
	for i in range (0,len(Elemek.karakterek)):
		print(Elemek.karakterek[i], Elemek.mennyiség[i], Elemek.kód[i], i)

def isLeaf(root):
	return root.left is None and root.right is None

class Node:
	def __init__(self, ch, freq, left=None, right=None):
		self.ch = ch
		self.freq = freq
		self.left = left
		self.right = right

	def __lt__(self, other):
		return self.freq < other.freq

def encode(root, str, huffman_code):

	if root is None:
		return

	if isLeaf(root):
		huffman_code[root.ch] = str if len(str) > 0 else '1'

	encode(root.left, str + '0', huffman_code)
	encode(root.right, str + '1', huffman_code)

def decode(root, index, str):

	if root is None:
		return index
	
	if isLeaf(root):
		doksi.write(root.ch)
		return index

	index = index + 1
	root = root.left if str[index] == '0' else root.right
	return decode(root, index, str)


def buildHuffmanTree(text):
	if len(text) == 0:
		return

	freq = {i: text.count(i) for i in set(text)}

	pq = [Node(k, v) for k, v in freq.items()]
	heapq.heapify(pq)

	while len(pq) != 1:

		left = heappop(pq)
		right = heappop(pq)

		total = left.freq + right.freq
		heappush(pq, Node(None, total, left, right))

	root = pq[0]

	huffmanCode = {}
	encode(root, "", huffmanCode)
	átlagos_kódhossz=0
	for i in Elemek.karakterek:
		átlagos_kódhossz+=(len(huffmanCode[i]))*Elemek.előfordulás[Elemek.karakterek.index(i)]
	doksi.write("\n"+str(huffmanCode))
	

	string = ""
	for c in text:
		string += huffmanCode.get(c)
		
	doksi.write("\n"+string+"\n")
	if isLeaf(root):
		while root.freq > 0:
			print(root.ch, end='')
			root.freq = root.freq - 1
	else:
		index = -1
		while index < len(string) - 1:
			index = decode(root, index, string)
	doksi.write("\nAtlagos kodhossz: "+str(átlagos_kódhossz))
			
	
def main():
	üzenet=input()
	if üzenet=="rand":
		üzenet=[]
		doksi.write("Veletlen jelek: \n")
		for i in range(0,40):
			üzenet.append(random.randint(97,122))   

			doksi.write(str(üzenet[i])+", ")
			üzenet[i]=(chr(üzenet[i]))
		
	text=üzenet
	doksi.write("\n\nVeletlen jelek ASCII karakterei: ")
	doksi.write("\n"+str(üzenet))
	előfordulás=Counter(üzenet)
	for i in előfordulás:
		Elemek.karakterek.append(i)
		Elemek.mennyiség.append(előfordulás[i])
		
###############################################  SZINGULÁRIS	##
		
	for i in üzenet:
		if ord(i)+singular_key>122:
			singular.append(chr(ord(i)+singular_key-122+96))
		else:
			singular.append(chr(ord(i)+singular_key))

################################################################

########################################## AZONOS HOSSZÚSÁGÚ KÓD
	 
	rnd=random.sample(range(1024, 2047), len(Elemek.karakterek))
	x=0
	for i in Elemek.karakterek:
		rnd[x]=bin(rnd[x])
		Elemek.kód.append(rnd[x][2:])
		x+=1

################################################################
	
	# print_object() ## elem objektum kiírása
	doksi.write("\n\nElofordulasok: \n")
	for i in range(0,len(Elemek.karakterek)):
		doksi.write("\n"+ str(Elemek.karakterek[i])+" "+str(Elemek.mennyiség[i])+" db")
		Elemek.előfordulás.append(Elemek.mennyiség[i]/len(üzenet))
	doksi.write("\n\nRelativ gyakorisag: \n")
	for i in range(0,len(Elemek.karakterek)):
		doksi.write("\n"+ str(Elemek.karakterek[i])+" "+str(Elemek.előfordulás[i]))
		Elemek.informácio_mennyiség.append(math.log2(1/Elemek.előfordulás[i]))
	doksi.write("\nInformacio mennyiseg: \n")
	for i in range(0,len(Elemek.karakterek)):
		doksi.write("\n"+ str(Elemek.karakterek[i])+" "+str(Elemek.informácio_mennyiség[i]))
		Elemek.entrópia+=Elemek.mennyiség[i]*Elemek.informácio_mennyiség[i]
	doksi.write("\nEntropia: "+str(Elemek.entrópia))
	
	doksi.write("\nSzingularis kod:")
	doksi.write("\n"+str(singular))
	doksi.write("\nAzonos hosszusagu kod:")
	doksi.write("\n"+str(Elemek.kód))
################################################### HUFFMANN KÓD
	doksi.write("\nHuffmann kod:")
	buildHuffmanTree(text)
	# Elemek.előfordulás.sort()
	# print(Elemek.előfordulás, "\n")
	# print(Elemek.karakterek, "\n")
	# Elemek.előfordulás,Elemek.karakterek= zip(*sorted(zip(Elemek.előfordulás, Elemek.karakterek)))
	# print(Elemek.előfordulás, "\n")
	# print(Elemek.karakterek, "\n")
	# Elemek.előfordulás=list(Elemek.előfordulás)
	# x=0
	# kod=[]
	# while(len(Elemek.előfordulás)>1):
	#	 for i in range(len(kod)):
	#		kod.append
	#	 Elemek.előfordulás[0]+=(Elemek.előfordulás[1])
	#	 # print("nem rendezet:",Elemek.előfordulás,"\n")
	#	 Elemek.előfordulás.sort()
	#	 # print("rendezet:",Elemek.előfordulás,"\n")
	#	 del Elemek.előfordulás[0]
	#	 print(Elemek.előfordulás,"\n")
	#	 print(x)
	#	 x+=1
	 	
################################################################

if __name__ == "__main__":
	main()
