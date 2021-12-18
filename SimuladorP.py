from numpy import sin, sign, clip
from json import dump

pi = 3.141592653589793

class Controller:
	def __init__(self):
		self.SP = 0
		self.kP = 0.75

	def update(self, phi):
		return (self.SP - phi) * self.kP


class InvPendulum:
	def __init__(self, phi, omega, actuatorStrength, gravityStrength, atenuation): # Definición del péndulo
		self.phi = phi # Ángulo respecto a la horizontal
		self.omega = omega # Velocidad angular
		self.actuatorStrength = actuatorStrength # Momento/Inercia (aceleración angular) de los actuadores
		self.gravityStrength = gravityStrength # Aceleración de la gravedad entre la longitud
		self.controlSignal = 0 # Señal entre -1 y 1 que controla los actuadores
		self.atenuation = atenuation # Pérdida a la fricción

	def tick(self): # Pasa un "tick" de tiempo
		self.phi = clip(self.phi + self.omega, -pi/2, pi/2) # Actualizar phi (limitado entre -90º y 90º)
		self.omega = self.omega * self.atenuation
		self.omega = self.omega + sin(self.phi) * self.gravityStrength + self.actuatorStrength * self.controlSignal # Actualizar omega

	def control(self, controlSignal):
		self.controlSignal = clip(controlSignal, -1, 1) # Actualiza la señal de control (limitada a -1 y 1)

	def printState(self):
		print(f"φ: {self.phi}\n ({self.phi/pi*180}º)")
		print(f"ω: {self.omega}\n ({self.omega/pi*180}º)")
		print(f"c: {self.controlSignal}\n")

	def getState(self): # Devuelve todo el estado
		return [self.phi,self.omega,self.controlSignal]


OVO = InvPendulum(phi=-0.1, omega=0, actuatorStrength=0.15, gravityStrength=0.1, atenuation=0.85)
controllerP = Controller()

log = []

for a in range(200):
	OVO.printState()
	OVO.control(controllerP.update(OVO.getState()[0]))
	OVO.tick()
	log.append(OVO.getState())

with open("controladorP.json", "w") as outfile:
	dump(log, outfile)