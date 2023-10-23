from JOGADOR import ROBO
from JOGADOR import Player

timestep = 32 #Passo de simulação:
recursos = Player(timestep)
estado_partida = "P"
ROBOS = ROBO(recursos)

while recursos.step(timestep) != -1:
    recursos.updateAll()
    ROBOS.strategy_preview()

#Looping principal:
# while robo.step(timestep) != -1:
#     robo.updateAll()
    
#     if(robo.index == 1):
#         if(robo.match_state=='P'):
#             robo.hold()
#         elif(robo.match_state=='Q'):
#             RE1.defesa(robo)
#             REModo = "D" 
#         elif(robo.match_state=='W'):
#             RE1.ataque(robo)
#             REModo = "A" 
#         elif(robo.match_state=='O'):
           
#             RE1.jogo(robo, REModo)
#         elif(robo.match_state=='G'):
#             RE1.gol(robo)
            
#     elif(robo.index == 2):
#         if(robo.match_state=='P'):
#             robo.hold()
#         elif(robo.match_state=='Q'):
#             RE2.defesa(robo)
#             REModo = "D" 
#         elif(robo.match_state=='W'):
#             RE2.ataque(robo)
#             REModo = "A"            
#         elif(robo.match_state=='O'):
            
#             RE2.jogo(robo, REModo)
#         elif(robo.match_state=='G'):
#             RE2.gol(robo)
            
#     elif(robo.index == 3):
#         if(robo.match_state=='P'):
#             robo.hold() 
#         elif(robo.match_state=='Q'):
#             RD1.ataque(robo)
#             RDModo = "A"

#         elif(robo.match_state=='W'):
#             RD1.defesa(robo)
#             RDModo = "D"

#         elif(robo.match_state=='O'):
#             RD1.jogo(robo)
            
#         elif(robo.match_state=='G'):
#             RD1.gol(robo)
            
#     elif(robo.index == 4):
#         if(robo.match_state=='P'):
#             robo.hold() 
#         elif(robo.match_state=='Q'):
#             RD2.ataque(robo)
#         elif(robo.match_state=='W'):
#             RD2.defesa(robo)
#         elif(robo.match_state=='O'):
#             RD2.jogo(robo)
#         elif(robo.match_state=='G'):
#             RD2.gol(robo)