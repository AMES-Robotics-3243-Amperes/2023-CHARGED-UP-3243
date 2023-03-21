import json



class HSVValue:
	
	H : int
	S : int
	V : int

	def __init__(this, H : int, S : int, V : int):
		this.H = H
		this.S = S
		this.V = V


class Constants:
	unit:int = 5

	shortPulseLength:int = 1*unit
	longPulseLength:int = 3*unit

	spacePulseLength:int = 1*unit
	spaceCharLength:int = 3*unit
	spaceWordLength:int = 7*unit

	endPauseLength:int = 100

	sort_keys:bool = True
	indent:int = 4

	offStrandState:list[HSVValue] = [
		HSVValue(0, 0, 0)
	]

	onStrandState:list[HSVValue] = [
		HSVValue(0, 0, 255)
	]


class Morse:

	charToMorseDict : dict[str, tuple[bool]] = {
		"a" : (0, 1),
		"b" : (1, 0, 0, 0),
		"c" : (1, 0, 1, 0),
		"d" : (1, 0, 0),
		"e" : tuple([0]),
		"f" : (0, 0, 1, 0),
		"g" : (1, 1, 0),
		"h" : (0, 0, 0, 0),
		"i" : (0, 0),
		"j" : (0, 1, 1, 1),
		"k" : (1, 0, 1),
		"l" : (0, 1, 0, 0),
		"m" : (1, 1),
		"n" : (1, 0),
		"o" : (1, 1, 1),
		"p" : (0, 1, 1, 0),
		"q" : (1, 1, 0, 1),
		"r" : (0, 1, 0),
		"s" : (0, 0, 0),
		"t" : tuple([1]),
		"u" : (0, 0, 1),
		"v" : (0, 0, 0, 1),
		"w" : (0, 1, 1),
		"x" : (1, 0, 0, 1),
		"y" : (1, 0, 1, 1),
		"z" : (1, 1, 0, 0),

		"1" : (0, 1, 1, 1, 1),
		"2" : (0, 0, 1, 1, 1),
		"3" : (0, 0, 0, 1, 1),
		"4" : (0, 0, 0, 0, 1),
		"5" : (0, 0, 0, 0, 0),
		"6" : (1, 0, 0, 0, 0),
		"7" : (1, 1, 0, 0, 0),
		"8" : (1, 1, 1, 0, 0),
		"9" : (1, 1, 1, 1, 0),
		"0" : (1, 1, 1, 1, 1),

		"?" : (0, 0, 1, 1, 0, 0),
		"!" : (1, 0, 1, 0, 1, 1),
		"." : (0, 1, 0, 1, 0, 1),
		"," : (1, 1, 0, 0, 1, 1),
		";" : (1, 0, 1, 0, 1, 0),
		":" : (1, 1, 1, 0, 0, 0),
		"+" : (0, 1, 0, 1, 0),
		"-" : (1, 0, 0, 0, 0, 1),
		"/" : (1, 0, 0, 1, 0),
		"=" : (1, 0, 0, 0, 1),
	}

	morseToCharDict : dict[tuple[bool], str] = dict([(pair[1], pair[0]) for pair in charToMorseDict.items()])

	signals : tuple[bool, ...]

	def __init__(this, *signals : bool):
		this.signals = signals
	
	
	def fromChar(char : str):
		return __class__(*__class__.charToMorseDict[char])
	

	def getChar(this) -> str:
		return __class__.morseToCharDict[this.signals]
	
	def getTimings(this) -> list[int]:
		output : list[int] = []

		for i, signal in enumerate(this.signals):
			if signal == 1:
				output += [1]*Constants.longPulseLength
			else:
				output += [1]*Constants.shortPulseLength
			
			if i < len(this.signals) - 1:
				output += [0]*Constants.spacePulseLength
		
		return output




class MorseWord:

	morseChars : tuple[Morse, ...]

	def __init__(this, *morseChars : Morse):
		this.morseChars = morseChars
	
	def fromString(wordString:str):
		morseChars : list[Morse] = []
		for char in wordString:
			morseChars.append(Morse.fromChar(char))
		
		return __class__(*morseChars)
	
	def getString(this):
		output : str = ""

		for morseChar in this.morseChars:
			output += morseChar.getChar()
		
		return output
	
	def getTimings(this) -> list[int]:
		output : list[int] = []

		for i, morseChar in enumerate(this.morseChars):
			output += morseChar.getTimings()
			
			if i < len(this.morseChars) - 1:
				output += [0]*Constants.spaceCharLength
		
		return output



class MorseSequence:

	morseWords : tuple[MorseWord, ...]

	def __init__(this, *morseWords : MorseWord):
		this.morseWords = morseWords
	
	def fromString(text:str):
		morseWords : list[MorseWord] = []
		for word in text.split(" "):
			morseWords.append(MorseWord.fromString(word))
		
		return __class__(*morseWords)
	
	def getString(this):
		output : str = ""

		for i, morseWord in enumerate(this.morseWords):
			output += morseWord.getString()

			if i < len(this.morseWords) - 1:
				output += " "
		
		return output
	
	def getTimings(this) -> list[int]:
		output : list[int] = []

		for i, morseWord in enumerate(this.morseWords):
			output += morseWord.getTimings()
			
			if i < len(this.morseWords) - 1:
				output += [0]*Constants.spaceWordLength
			else:
				output += [0]*Constants.endPauseLength
		
		return output



class Pattern:

	strandStates : list[list[HSVValue]]
	strandStateIndexes : list[int]

	def __init__(this, strandStates : list[list[HSVValue]], strandStateIndexes : list[int]):
		this.strandStates = strandStates
		this.strandStateIndexes = strandStateIndexes
	
	def default(self, o):
		return self.__dict__






textToDisplay = input("What text do you want to display? ")

codeToDisplay = MorseSequence.fromString(textToDisplay)

jsonObject = Pattern([Constants.offStrandState, Constants.onStrandState], codeToDisplay.getTimings())

humanReadableInput = input("Use human readable format? (Y/n) ").lower()

humanReadable:bool = True
if humanReadableInput in {"n", "no"}:
	humanReadable = False

if humanReadable:
	print(json.dumps(jsonObject, default=lambda o:o.__dict__, sort_keys=Constants.sort_keys, indent=Constants.indent))
else:
	print(json.dumps(jsonObject, default=lambda o:o.__dict__))

writeToFileInput = input("Write to file? (y/N) ").lower()

writeToFile:bool = False
if writeToFileInput in {"y", "yes"}:
	writeToFile = True


if writeToFile:
	filename = input("What filename? ")
	if filename[-5:] != ".json":
		filename += ".json"
	
	with open(filename, "w") as file:
		if humanReadable:
			json.dump(jsonObject, file, default=lambda o:o.__dict__, sort_keys=Constants.sort_keys, indent=Constants.indent)
		else:
			json.dump(jsonObject, file, default=lambda o:o.__dict__)
