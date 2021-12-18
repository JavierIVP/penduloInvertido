from numpy import sin, sign, clip
from random import uniform as randFloat
from openpyxl import Workbook
from openpyxl.chart import (LineChart, Reference,)

import multiprocessing as mp

pi = 3.141592653589793
minSpeed = 0.024525
maxSpeed = 1.1772
timeConstant = 0.001

def map(n, minN, maxN, minR, maxR):
	return (n-minN)/(maxN-minN)*(maxR-minR)+minR

class OVO:
	def __init__(self, phiX, phiY, omegaX, omegaY, mass, momentOfInertia, height, gravity, frictionCoefficient, fanAcceleration): # Definición del péndulo
		self.phiX = phiX # Ángulo respecto a la horizontal
		self.phiY = phiY # Ángulo respecto a la vertical
		self.omegaX = omegaX # Velocidad angular
		self.omegaY = omegaY
		self.mass = mass
		self.mOI = momentOfInertia # Momento/Inercia (aceleración angular) de los actuadores
		self.height = height
		self.gravity = gravity # Aceleración de la gravedad entre la longitud
		self.controlSignals = [minSpeed,minSpeed,minSpeed,minSpeed] # Señal entre -1 y 1 que controla los actuadores
		self.fC = frictionCoefficient # Pérdida a la fricción
		self.fanSpeeds = [minSpeed,minSpeed,minSpeed,minSpeed]
		self.fanAcc = fanAcceleration

	def tick(self): # Pasa un "tick" de tiempo
		self.phiX = clip(self.phiX + self.omegaX*timeConstant, -pi/2, pi/2) # Actualizar phi (limitado entre -90º y 90º)
		self.phiY = clip(self.phiY + self.omegaY*timeConstant, -pi/2, pi/2)

		self.omegaX -= sign(self.omegaX) * self.omegaX**2 * self.fC *timeConstant
		self.omegaX += (0.166*(self.fanSpeeds[0]-self.fanSpeeds[1]-self.fanSpeeds[2]+self.fanSpeeds[3])+self.height*self.gravity*sin(self.phiX))/self.mOI *timeConstant
		
		self.omegaY -= sign(self.omegaY) * self.omegaY**2 * self.fC *timeConstant
		self.omegaY += (0.166*(self.fanSpeeds[0]+self.fanSpeeds[1]-self.fanSpeeds[2]-self.fanSpeeds[3])+self.height*self.gravity*sin(self.phiY))/self.mOI *timeConstant
		
		for i in range(4):
			self.fanSpeeds[i] = self.fanSpeeds[i] + clip(self.controlSignals[i]-self.fanSpeeds[i],-self.fanAcc*timeConstant,self.fanAcc*timeConstant)


	def control(self, controlSignals):
		for i in range(4):
			self.controlSignals[i] = map(clip(controlSignals[i], 0, 1),0,1,minSpeed,maxSpeed) # Actualiza la señal de control (limitada entre 0 y 1)

	def printState(self):
		print(f"φx: {self.phiX}\n ({self.phiX/pi*180}º)")
		print(f"ωx: {self.omegaX}\n ({self.omegaX/pi*180}º)")
		print(f"φy: {self.phiY}\n ({self.phiY/pi*180}º)")
		print(f"ωy: {self.omegaY}\n ({self.omegaY/pi*180}º)")
		print(f"cx: {self.fanSpeeds[0]-self.fanSpeeds[1]-self.fanSpeeds[2]+self.fanSpeeds[3]}")

	def getState(self): # Devuelve todo el estado
		# Formato: [phiX, phiY, omegaX, omegaY, control]
		return [self.phiX,self.phiY,self.omegaX,self.omegaY,self.fanSpeeds]

class ControllerP:
	def __init__(self, SPx, SPy, kP):
		self.SPx = SPx
		self.SPy = SPy
		self.kP = kP

	def update(self, state):
		xFactor = (self.SPx-state[0])*self.kP
		yFactor = (self.SPy-state[1])*self.kP
		return [+xFactor+yFactor, -xFactor+yFactor, -xFactor-yFactor, +xFactor-yFactor]

class ControllerPD:
	def __init__(self, SPx, SPy, kP, kD):
		self.SPx = SPx
		self.SPy = SPy
		self.kP = kP
		self.kD = kD

	def update(self, state):
		xFactor = (self.SPx-state[0])*self.kP - state[2]*self.kD
		yFactor = (self.SPy-state[1])*self.kP - state[3]*self.kD
		return [+xFactor+yFactor, -xFactor+yFactor, -xFactor-yFactor, +xFactor-yFactor]

class ControllerPID:
	def __init__(self, SPx, SPy, kP, kI, kD):
		self.SPx = SPx
		self.SPy = SPy
		self.kP = kP
		self.kD =kD
		self.kI = kI
		self.cIX = 0
		self.cIY = 0

	def update(self, state):
		self.cIX += (self.SPx-state[0])*timeConstant
		xFactor = (self.SPx-state[0])*self.kP - state[2]*self.kD + self.cIX * self.kI
		self.cIY += (self.SPy-state[1])*timeConstant
		yFactor = (self.SPy-state[1])*self.kP - state[3]*self.kD + self.cIY * self.kI
		return [+xFactor+yFactor, -xFactor+yFactor, -xFactor-yFactor, +xFactor-yFactor]

class ControllerFisico:
	def __init__(self, SPx, SPy, kP, k):
		self.SPx = SPx
		self.SPy = SPy
		self.kP = kP
		self.k = k

	def update(self, state):
		xFactor = (self.k*(self.SPx-state[0]) - state[2])*self.kP
		yFactor = (self.k*(self.SPy-state[1]) - state[3])*self.kP
		return [+xFactor+yFactor, -xFactor+yFactor, -xFactor-yFactor, +xFactor-yFactor]


def perform(ovo, controller, loopLength, logAmmount = 200, length=20000):

	log = [[],[],[],[],[],[],[0]]

	for a in range(length):
		state = ovo.getState()
		if a%int(length/logAmmount) == 0:
			for a in range(6):
				log[a].append((state[:-1] + [controller.SPx, controller.SPy])[a])
			log[6].append(log[6][-1]+(state[4][0]+state[4][1]+state[4][2]+state[4][3])/length*5)
		ovo.tick()
		if a%loopLength == 0:
			ovo.control(controller.update([state[0]*randFloat(0.99,1.01)+state[2]*randFloat(0.01,0.05),state[1]*randFloat(0.99,1.01)+state[3]*randFloat(0.01,0.05),state[2]*randFloat(0.99,1.01),state[3]*randFloat(0.99,1.01)]))
	return log

def makeWorksheet(wb, name, logAmmount, length, phiX, phiY, oX, oY, m, mOI, h, g, fC, fAcc, SPx=0, SPy=0, kP=0.78, kI=0.005, kD=0.005):
	
	print(f"Starting {name}")

	qP = perform(OVO(phiX, phiY, oX, oY, m, mOI, h, g, fC, fAcc), controllerP, 3, logAmmount, length,)
	print("  - P done")
	qPD = perform(OVO(phiX, phiY, oX, oY, m, mOI, h, g, fC, fAcc), controllerPD, 4, logAmmount, length,)
	print("  - PD done")
	qPID = perform(OVO(phiX, phiY, oX, oY, m, mOI, h, g, fC, fAcc), controllerPID, 5, logAmmount, length,)
	print("  - PID done")
	qF = perform(OVO(phiX, phiY, oX, oY, m, mOI, h, g, fC, fAcc), controllerFisico, 4, logAmmount, length,)
	print("  - F done")

	worksheet = wb.create_sheet(name)

	worksheet.append(["P"])
	for group in qP:
		worksheet.append(group)
	chart = LineChart()
	chart.add_data(Reference(worksheet, 1, 2, logAmmount, 2), "PV")
	chart.add_data(Reference(worksheet, 1, 6, logAmmount, 6), "SP")
	chart.add_data(Reference(worksheet, 1, 8, logAmmount, 8), "Energy")
	worksheet.add_chart(chart, "B2")
	
	worksheet.append(["PD"])
	for group in qPD:
		worksheet.append(group)
	chart = LineChart()
	chart.add_data(Reference(worksheet, 1, 10, logAmmount, 10), "PV")
	chart.add_data(Reference(worksheet, 1, 14, logAmmount, 14), "SP")
	chart.add_data(Reference(worksheet, 1, 16, logAmmount, 16), "Energy")
	worksheet.add_chart(chart, "K2")

	worksheet.append(["PID"])
	for group in qPID:
		worksheet.append(group)
	chart = LineChart()
	chart.add_data(Reference(worksheet, 1, 18, logAmmount, 18), "PV")
	chart.add_data(Reference(worksheet, 1, 22, logAmmount, 22), "SP")
	chart.add_data(Reference(worksheet, 1, 24, logAmmount, 24), "Energy")
	worksheet.add_chart(chart, "B17")

	worksheet.append(["F"])
	for group in qF:
		worksheet.append(group)
	chart = LineChart()
	chart.add_data(Reference(worksheet, 1, 26, logAmmount, 26), "PV")
	chart.add_data(Reference(worksheet, 1, 30, logAmmount, 30), "SP")
	chart.add_data(Reference(worksheet, 1, 32, logAmmount, 32), "Energy")
	worksheet.add_chart(chart, "K17")


if __name__ == "__main__":

	momentOfInertia = 0.077 * 0.07**2 + 0.085 * 0.06**2 + 4 * 0.052 * 0.166**2 + 4 * 0.011 * 0.011**2 + 0.340 * 0.1**2 + 0.078 * 0.01**2 /2
	fanAcceleration = (maxSpeed-minSpeed)/0.136
	mass = 0.838

	workbook = Workbook()

	controllerP = ControllerP(0,0,1.4)
	controllerPD = ControllerPD(0,0,2.5,0.2)
	controllerPID = ControllerPID(0,0,2.1,1.7,0.003)
	controllerFisico = ControllerFisico(0,0,3.8,2.7)
	makeWorksheet(workbook, "normal",400,8000,phiX=0.1,phiY=0.1,oX=0,oY=0,m=mass,mOI=momentOfInertia,h=0.03,g=9.81,fC=1,fAcc=fanAcceleration)
	
	controllerP = ControllerP(0,0,1.4)
	controllerPD = ControllerPD(0,0,2.5,0.2)
	controllerPID = ControllerPID(0,0,2.1,1.7,0.003)
	controllerFisico = ControllerFisico(0,0,3.8,2.7)
	makeWorksheet(workbook, "high friction",400,8000,phiX=0.1,phiY=0.1,oX=0,oY=0,m=mass,mOI=momentOfInertia,h=0.03,g=9.81,fC=15,fAcc=fanAcceleration)
	
	controllerP = ControllerP(0,0,1.4)
	controllerPD = ControllerPD(0,0,2.5,0.2)
	controllerPID = ControllerPID(0,0,2.1,1.7,0.003)
	controllerFisico = ControllerFisico(0,0,3.8,2.7)
	makeWorksheet(workbook, "vacuum",400,12000,phiX=0.1,phiY=0.1,oX=0,oY=0,m=mass,mOI=momentOfInertia,h=0.03,g=9.81,fC=0,fAcc=fanAcceleration)
	

	controllerP = ControllerP(0,0,1.4)
	controllerPD = ControllerPD(0,0,2.5,0.2)
	controllerPID = ControllerPID(0,0,2.1,1.7,0.003)
	controllerFisico = ControllerFisico(0,0,3.8,2.7)
	makeWorksheet(workbook, "highG",400,12000,phiX=0.1,phiY=0.1,oX=0,oY=0,m=mass,mOI=momentOfInertia,h=0.03,g=12,fC=5,fAcc=fanAcceleration,kP=1)
	
	controllerP = ControllerP(0.1,0,1.4)
	controllerPD = ControllerPD(0.1,0,2.1,0.4)
	controllerPID = ControllerPID(0.1,0,2.1,0.5,0.02)
	controllerFisico = ControllerFisico(0.1,0,4,3)
	makeWorksheet(workbook, "offsetPreparado",400,20000,phiX=-0.1,phiY=0.1,oX=0,oY=0,m=mass,mOI=momentOfInertia,h=0.03,g=9.81,fC=5,fAcc=fanAcceleration)

	del workbook["Sheet"]
	workbook.save(filename=f"data.xlsx")