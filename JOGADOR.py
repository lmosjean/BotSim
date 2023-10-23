from random import randint
import numpy as np
from controller import Robot, Motor, PositionSensor, Supervisor, Node, Emitter, Receiver, Keyboard, Connector
def sig(x):
    return (x/abs(x))

#Função de saturação a partir de um limiar:
def sat(x, threshold):
    if(abs(x) < threshold):
        return x
    else:
        return (threshold*sig(x))


class Player(Supervisor, Robot, Node):
    #Método de inicialização da instância:
    def __init__(self, timestep):
        #Supervisor:
        super(Player, self).__init__()
        #Obtendo o DEF do nó do robô:
        self.index = (int(self.getName()[6]))
        robo_defs = ['RE1', 'RE2', 'RD1', 'RD2']
        self.this_node = self.getFromDef(robo_defs[self.index-1])
        self.ts = timestep
        #Obtendo o gol de ataque da equipe:
        if(self.index<=2):
            self.gt = [2300, 0]
        else:
            self.gt = [-2300, 0]
        #Teclado:
        self.kb = self.getKeyboard()
        self.kb.enable(self.ts)
        #Motor:
        self.MTD = self.getDevice('right_motor')
        self.MTE = self.getDevice('left_motor')
        self.MTD.setVelocity(0.0)
        self.MTE.setVelocity(0.0)
        self.MTD.setPosition(float('+inf'))
        self.MTE.setPosition(float('+inf'))
        #Chutador:
        self.foot = self.getDevice('foot_motor')
        self.foot_enc = self.getDevice('foot_position_sensor')
        self.foot_enc.enable(self.ts)
        self.fs = self.getFromDef('kicker_solid_'+self.getName()[6])
        #Conector:
        self.con = self.getDevice('connector')
        self.con.enablePresence(self.ts)
        #Rádio:
        self.emitter = self.getDevice('emitter')
        self.receiver = self.getDevice('receiver')
        self.arena_receiver = self.getDevice('arena_receiver')
        self.receiver.enable(self.ts)
        self.arena_receiver.enable(self.ts)
        #Obtendo pose dos robôs:
        self.this_pose = []
        self.duo = []
        self.adv = []
        self.ball = []
        #Distâncias entre os robôs:
        self.dant = [0, 0, 0]
        #Estado da função setPose:
        self.sp_state = 1
        #Estado da partida:
        self.match_state = 'P'
        #Estado da função de passar:
        self.passmode = 1
        #Variável de posse instantânea:
        self.instposs = False
        #Tempo de posse:
        self.tposs = 0
        #Booleana de reset de posição na pathTracker:
        self.reset = True
        #Booleana do moveBy:
        self.mb = False
        #Tempo do moveBy:
        self.mbt = 0
        #Step para a função de randomShoot:
        self.rs_step = 1
        #Posição y aleatória para chute:
        self.randy = 0
        #Estado da função de receber:
        self.receivemode = 1
        #Posição anterior da bola:
        self.posball = [0, 0]
        #Definição de constantes:
        self.const = {
            'VMAX' : 15.0,
            'VROT' : 10.0,
            'offset' : 40,
            'LR': 30,
            'LT': 8,
            'KR' : (1/30),
            'KT' : (1/(8+40)),
            'RP' : 1,
            'TP' : 5,
            'KA' : 90,
            'LC' : 350
        }
    
    #Função para obter/atualizar as poses:
    def getPoses(self, node_defs):
        #Atualizar a pose do robô:
        posei = self.this_node.getPose()
        self.this_pose = [round((posei[3]*1e3), 2), round((posei[7]*1e3), 2), round(np.degrees(np.arctan2(posei[4], posei[5])), 2)]
        #Obter poses externas desejadas:
        poses = []
        for i in node_defs:
            try:
                #Obtem o nó especificado por DEF:
                node = self.getFromDef(i)
                #Obtem a matriz de transformação homogênea:
                posei = node.getPose()
                #Obtem o vetor pose com 3GDL's (x, y, theta) [mm, mm, °]:
                poses.append([round((posei[3]*1e3), 2), round((posei[7]*1e3), 2), round(np.degrees(np.arctan2(posei[4], posei[5])), 2)])
            except:
                pass
        return poses
        
    #Função para pegar o estado da partida:
    def matchStateUpdate(self):
        if(self.arena_receiver.getQueueLength() > 0):
            code = self.arena_receiver.getBytes().decode();
            self.match_state = code
            self.arena_receiver.nextPacket();
            return self.match_state
            
    def updateAll(self):
        self.matchStateUpdate()
        posesa = self.getPoses(['BOLA', 'RE1', 'RE2', 'RD1', 'RD2'])
        self.ball = posesa[0]
        robos = posesa[1:]
        self.adv = []
        if(len(robos)>1):
            try:
                if(self.index%2):
                    self.duo = robos[self.index]
                else:
                    self.duo = robos[self.index-2]
            except:
                self.duo = None
            for i in robos:
                if((i != self.this_pose) and (i != self.duo)):
                    self.adv.append(i)
    
    #Função para parar o robô:
    def hold(self):
        self.MTD.setVelocity(0)
        self.MTE.setVelocity(0)
            
    #Função para setar um ângulo absoluto desejado:
    def setAngle(self, ang):
        #ângulo absoluto do robô:
        theta = self.this_pose[2]
        #Erro de rotação:
        eccw = abs(ang-theta)
        if(eccw>180):
            eccw = (360-eccw)
        ecw = (360-eccw)
        if(eccw<ecw):
            if(self.index<=2):
                sn = -1
            else:
                sn = 1
            ER = eccw
        else:
            if(self.index<=2):
                sn = 1
            else:
                sn = -1
            ER = ecw
        #Define a condição de continuar:
        go = (abs(ER) > self.const['RP'])
        #Verifica se o resultado ainda não está aceitável:
        if(go):
            #Correção brusca por limiar:
            if(abs(ER) > (self.const['LR'])):
                vd = (-self.const['VROT']*sn)
                ve = (self.const['VROT']*sn)
            else: #Ajuste proporcional final:
                vd = (-self.const['KR']*ER*self.const['VROT']*sn)
                ve = (self.const['KR']*ER*self.const['VROT']*sn)
        else: #Desliga os motores quando o alinhamento estiver satisfatório:
            vd = 0
            ve = 0
        #Associa as velocidades aos motores:
        self.MTD.setVelocity(vd)
        self.MTE.setVelocity(ve)
        return go
    
    #Função para alinhar a um alvo especificado:
    def align(self, target):
        #Obtendo os diferenciais das coordenadas x e y:
        dx = (target[0]-self.this_pose[0])
        dy = (target[1]-self.this_pose[1])
        #Obtendo o ângulo robô-alvo:
        alpha = np.degrees(np.arctan2(dy, dx))
        #ângulo absoluto do robô:
        theta = self.this_pose[2]
        #Erro de rotação:
        eccw = abs(alpha-theta)
        if(eccw>180):
            eccw = (360-eccw)
        ecw = (360-eccw)
        if(eccw<ecw):
            if(self.index<=2):
                sn = -1
            else:
                sn = 1
            ER = eccw
        else:
            if(self.index<=2):
                sn = 1
            else:
                sn = -1
            ER = ecw
        #Define a condição de continuar:
        go = (abs(ER) > self.const['RP'])
        #Verifica se o alinhamento ainda não está aceitável:
        if(go):
            #Correção brusca por limiar:
            if(abs(ER) > (self.const['LR'])):
                vd = (-self.const['VROT']*sn)
                ve = (self.const['VROT']*sn)
            else: #Ajuste proporcional final:
                vd = (-self.const['KR']*ER*self.const['VROT']*sn)
                ve = (self.const['KR']*ER*self.const['VROT']*sn)   
        else: #Desliga os motores quando o alinhamento estiver satisfatório:
            vd = 0
            ve = 0
        #Associa as velocidades aos motores:
        self.MTD.setVelocity(vd)
        self.MTE.setVelocity(ve)
        return go
    
    #Função para ir até um alvo especificado:
    def gotoPos(self, target, tillcon=False):
        #Obtendo os diferenciais das coordenadas x e y:
        dx = (target[0]-self.this_pose[0])
        dy = (target[1]-self.this_pose[1])
        #Obtendo a distância até o alvo:
        dist = np.sqrt((dx**2)+(dy**2))
        #Obtendo o ângulo robô-alvo:
        alpha = np.degrees(np.arctan2(dy, dx))
        #ângulo absoluto do robô:
        theta = self.this_pose[2]
        #Erro de rotação:
        eccw = abs(alpha-theta)
        if(eccw>180):
            eccw = (360-eccw)
        ecw = (360-eccw)
        if(eccw<ecw):
            ER = eccw
            if(self.index<=2):
                sn = -1
            else:
                sn = 1
        else:
            ER = ecw
            if(self.index<=2):
                sn = 1
            else:
                sn = -1
        #Erro de translação:
        ET = (dist - self.const['offset'])
        #Define a condição de continuar:
        if(tillcon):
            go = (not self.ballHold())
        else:
            go = (abs(ET) > self.const['TP'])
        #Verifica se o alinhamento e a posição ainda não estão aceitáveis:
        if(go):
            #Correção brusca por limiar:
            if(abs(ER) > self.const['LR']):
                vd = (-self.const['VROT']*sn)
                ve = (self.const['VROT']*sn)
            else: #Ajuste suave de rotação e posição simultâneas:
                vd = (sat((1-np.sin(np.radians(ER*sn))), 1) * (sig(ET)*np.sqrt(abs(sat((ET*self.const['KT']), 1)))) * self.const['VMAX'])
                ve = (sat((1+np.sin(np.radians(ER*sn))), 1) * (sig(ET)*np.sqrt(abs(sat((ET*self.const['KT']), 1)))) * self.const['VMAX'])
        else: #Desliga os motores quando o resultado estiver satisfatório:
            vd = 0
            ve = 0
        #Associa as velocidades aos motores:
        self.MTD.setVelocity(vd)
        self.MTE.setVelocity(ve)
        return go
    
    #Função para deixar o robô numa pose especificado:
    def setPose(self, target):
        #Obtendo os diferenciais das coordenadas x e y:
        dx = (target[0]-self.this_pose[0])
        dy = (target[1]-self.this_pose[1])
        #Obtendo o ângulo robô-alvo:
        angle = (np.pi - np.arctan2(dy, dx))
        #Adicionando o offset no vetor de aproximação do objeto:
        tg = [(target[0] - (self.const['offset']*np.cos(angle))), (target[1] + (self.const['offset']*np.sin(angle)))]
        if(self.sp_state==1):
            if(not self.gotoPos(tg)):
                self.sp_state = 2
        elif(self.sp_state==2):
            if(not self.setAngle(target[2])):
                    self.sp_state = 1
                    return False
        return True
    
    #Função para armar o chutador:
    def load(self):
        self.con.lock()
        self.foot.setVelocity(300)
        self.foot.setPosition(np.radians(-self.const['KA']))
        while self.step(self.ts) != -1:
            if((np.degrees(self.foot_enc.getValue())-(-self.const['KA'])) < 1e-1):
                break
        self.fs.resetPhysics()
        
    #Função para chutar:
    def kick(self, power):
        if((power > 0) and (power <= 100)):
            self.con.unlock()
            self.foot.setVelocity(power*10)
            self.foot.setPosition(np.radians(self.const['KA']))
            while self.step(self.ts) != -1:
                if((np.degrees(self.foot_enc.getValue())-self.const['KA']) < 1e-1):
                    break
            self.foot.setVelocity(300)
            self.foot.setPosition(0)
            while self.step(self.ts) != -1:
                if(abs(self.foot_enc.getValue()) < 1e-1):
                    break
            self.fs.resetPhysics()
            self.con.lock()
            
    #Função para verificar posse de bola:
    def ballHold(self):
        if(self.con.getPresence() == 1):
            if(self.instposs):
                if((self.getTime()-self.tposs)>0.5):
                    return True
            else:
                self.instposs = True
                self.tposs = self.getTime()
        elif(self.con.getPresence() == 0):
            self.instposs = False
        return False
        
    #Função para localizar a reposição de bola:
    def getRepos(self):
        #Pega as coordenadas x e y da bola, e sua distância até o centro
        x = self.ball[0]
        y = self.ball[1]
        cdist = np.sqrt((x**2) + (y**2))
        #Se estiver mais próxima do centro:
        if(cdist < (np.sqrt(2*(1000**2)/2))):
            return 0
        #Se estiver mais próximo das marcas dos quadrantes:
        if(x > 0):
            if(y>0):
                cod = 1
            else:
                cod = 4
        else:
            if(y>0):
                cod = 2
            else:
                cod = 3
        return cod               
    
    #Função para enviar mensagem:
    def tell(self, pack):
        self.emitter.send(pack)
    
    #Função para receber mensagem:
    def listen(self):
        if(self.receiver.getQueueLength() > 0):
            pack = self.receiver.getString();
            self.receiver.nextPacket();
            return pack
        else:
            return -1
            
    #Função para passar a bola:
    def passBall(self):
        if(self.ballHold()):
            if(self.passmode == 1):
                #Avisa a dupla que vai passar a bola:
                self.tell('A')
                self.passmode += 1
            elif(self.passmode == 2):
                #Alinha com a dupla:
                if(not self.align(self.duo)):
                    self.passmode += 1
            elif(self.passmode == 3):
                #Aguarda confirmação da dupla:
                if(self.listen() == 'B'):
                    self.passmode += 1
            elif(self.passmode == 4):
                #Confere alinhamento:
                if(not self.align(self.duo)):
                    self.passmode += 1
            elif(self.passmode == 5):
                #Aguarda confirmação de alinhamento da dupla:
                if(self.listen() == 'C'):
                    self.passmode += 1
            elif(self.passmode == 6):
                #Realiza o passe com força dependente da distância
                self.load()
                dist = np.sqrt(((self.duo[0]-self.this_pose[0])**2) + ((self.duo[1]-self.this_pose[1])**2))
                vkick = (dist*(100.0/5000.0))
                if(vkick > 100):
                    vkick = 100
                self.kick(vkick)
                self.tell('D') #Avisa que foi dado o chute
                self.passmode = 0
            if(self.passmode):
                return True
            else:
                self.passmode = 1
                return False
        else:
            self.passmode = 1
            return False
        
    #Função para receber a bola:
    def receiveBall(self):
        #Aguarda aviso da dupla:
        if(self.receivemode == 1):
            if(self.listen() == 'A'):
                self.receivemode += 1
        elif(self.receivemode == 2):
            #Confirma a condição favorável ao passe:
            self.tell('B')
            self.receivemode += 1
        elif(self.receivemode == 3):
            #Alinha com a dupla:
            if(not self.align(self.duo)):
                self.tell('C') #confirma alinhamento
                self.receivemode += 1
        elif(self.receivemode == 4):
            #Aguarda o passe:
            if(self.listen() == 'D'):
                self.receivemode += 1
        elif(self.receivemode == 5):
            #Vai de encontro para dominar a bola:
            if(not self.gotoPos(self.ball, True)):
                self.receivemode = 0
        if(self.receivemode):
            return True
        else:
            self.receivemode = 1
            return False
            
    #Movimentação vertical:
    def verticalMove(self, target):
        erro = (target-self.this_pose[1])
        go = (abs(erro) > self.const['TP'])
        if(go):
            vd = (sat((1+(np.cos(np.radians(self.this_pose[2]))*sig(erro))), 1)*(sig(erro)*np.sqrt(abs(sat((erro*self.const['KT']), 1))))*self.const['VMAX'])
            ve = (sat((1-(np.cos(np.radians(self.this_pose[2]))*sig(erro))), 1)*(sig(erro)*np.sqrt(abs(sat((erro*self.const['KT']), 1))))*self.const['VMAX'])
        else:
            vd = 0
            ve = 0
        #Associa as velocidades aos motores:
        self.MTD.setVelocity(vd)
        self.MTE.setVelocity(ve)
        return go
        
    #Análise de trajetória:
    def pathTracker(self, p0, lim=200):
        if(abs(self.this_pose[0]-self.ball[0]) < 1800):
            x0, y0 = self.posball
            x = self.ball[0]
            y = self.ball[1]
            m = ((y-y0)/(x-x0+1e-12))
            yf = (y0 + (m*(self.this_pose[0]-x0)))
            yf = sat(yf, lim)
            dant = (self.this_pose[0]-self.posball[0])
            dat = (self.this_pose[0]-self.ball[0])
            if(dat < dant):
                self.verticalMove(yf)
                self.reset = True
            else:
                self.hold()            
        else:
            if(self.reset):
                self.reset = self.setPose(p0)
        self.posball = self.ball[:2]
    
    #Verificar qual robô está mais próximo da bola:
    def nearestRobot(self):
        dists = []
        robots = [self.this_pose]
        try:
            robots.append(self.duo)
        except:
            pass
        try:
            robots.append(self.adv[0])
        except:
            pass
        try:
            robots.append(self.adv[1])
        except:
            pass
        for i in robots:
            try:
                #Obtendo os diferenciais das coordenadas x e y:
                dx = (self.ball[0]-i[0])
                dy = (self.ball[1]-i[1])
                #Obtendo a distância até o alvo:
                dist = np.sqrt((dx**2)+(dy**2))
                dists.append(dist)
            except:
                pass
        dists = np.array(dists)
        return np.where(dists==np.min(dists))[0][0]
    
    #Função para detectar colisão:
    def collide(self):
        dists = []
        robots = []
        try:
            robots.append(self.duo)
        except:
            pass
        try:
            robots.append(self.adv[0])
        except:
            pass
        try:
            robots.append(self.adv[1])
        except:
            pass
        #Detectar colisões nos limites laterais do campo:
        if((abs(abs(self.this_pose[0])-2450)<self.const['LC']) or (abs(abs(self.this_pose[1])-1650)<self.const['LC'])):
            return True
        for i in robots:
            try:
                #Obtendo os diferenciais das coordenadas x e y:
                dx = (i[0]-self.this_pose[0])
                dy = (i[1]-self.this_pose[1])
                #Obtendo a distância até o alvo:
                dist = np.sqrt((dx**2)+(dy**2))
                dists.append(dist)
            except:
                pass
        for i in range(len(dists)):
            if(dists[i]<self.const['LC']):
                self.dant = dists
                return True
        self.dant = dists
        return False
        
    #Função para andar no vetor de aproximação (avançar em linha reta por um tempo, em milisegundos):
    def moveBy(self, tim):
        if(self.mb):
            if((self.getTime()-self.mbt)<(tim*1e-3)):
                self.MTD.setVelocity(self.const['VMAX'])
                self.MTE.setVelocity(self.const['VMAX'])
            else:
                self.mb = False
                self.hold()
                return False
        else:
            self.mbt = self.getTime()
            self.mb = True
        return True
        
    #Função para chute aleatório ao gol:
    def randomShoot(self):
        if(self.rs_step == 1):
            self.randy = randint(-320, 320)
            self.rs_step += 1
        elif(self.rs_step == 2):
            if(not self.align([self.gt[0], self.randy])):
                self.rs_step += 1
        elif(self.rs_step == 3):
            if(not self.moveBy(300)):
                self.load()
                self.kick(100)
                self.rs_step = 1
                return False
        return True

class ROBO:

    def __init__ (self,recursos):
        self.recursos = recursos
        self.robo_tag= ['RE1', 'RE2', 'RD1', 'RD2']
        self.__tag = self.robo_tag[(self.recursos.index - 1)]
        self.position = 0 #[x,y,t]
        self.estado_partida = recursos.match_state
        self.estrategia = None
        self.alinhado = False
        self.lastangle = 0
    
    def updateROBO(self):
        data = self.recursos
        
        print(data.this_pose)
        print(data.duo)
        print(data.adv)
        print(data.ball)
     
    def updateData(self):
              pass
        #metodo para obter dados essenciais como posicao do jogador
        #posicao da bola, posicao de outros objetos

    def is_near_ball(self, ball_position, threshold=100):
     # Check if the robot is near the ball within a certain threshold
         robot_x, robot_y = self.position[:2]
         ball_x, ball_y = ball_position[:2]
         distance = ((robot_x - ball_x) ** 2 + (robot_y - ball_y) ** 2) ** 0.5
         return distance < threshold

    def roboStatus(self):
        print("----------------------")
        print(f"{self.__tag},{self.recursos.this_pose}")
        print(f"Duo:,{self.recursos.duo}")
        print(f"adv:,{self.recursos.adv}")
        print(f"ball:,{self.recursos.ball}")
        print("----------------------")
        
    def getTargetInfo(self,target):
        posBola = self.recursos.ball
        if target == "BOLA":
            return posBola
        
        

    def strategy_preview(self):
               
            #self.roboStatus()
            game_mode = self.recursos.match_state
            
            if(game_mode=='P'):
                print("P")
                self.recursos.hold()

            if(game_mode=='O'):
                print("O")
                self.recursos.hold()

            if(game_mode == 'Q'): 
                if self.__tag == 'RE1':
                    self.posicionardefesa()
                    RModo = "A"
                if self.__tag == 'RE2':
                    self.ataque()
                else:
                    self.estrategia = 'Default Strategy'
            else:
                self.estrategia = 'Default Strategy'
   
    #FailSafe : corrige robôs que estejam travados ou em colisão com outros objetos e jogadores 
    #StrategyPreview: Escolhe a melhor abordagem de acordo com parâmetros em tempo real

 
    def posicionardefesa(self):
        #print("----------------------")
        #print("Posicionando para defesa")
        target = self.getTargetInfo("BOLA")      
        #print(target)
        dx = (target[0]-self.recursos.this_pose[0])
        dy = (target[1]-self.recursos.this_pose[1])
        angle = (np.pi - np.arctan2(dy, dx))
        tg = [(target[0] - (self.recursos.const['offset']*np.cos(angle))), (target[1] + (self.recursos.const['offset']*np.sin(angle)))]
        #print(f"dx:{dx},dy:{dy}")
        #print(f"Angulo robo-alvo:{angle}, tangente: {tg}")
        #print("----------------------")
        
        theta = self.recursos.this_pose[2]
        #Erro de rotação:
        eccw = abs(tg-theta)

        if tg != self.lastangle:
            self.alinhado = False

        if(eccw>=180):
            eccw = (360-eccw)
            
        ecw = (360-eccw)
        if(eccw<ecw):
            if(self.recursos.index<=2):
                sn = -1
            else:
                sn = 1
            ER = eccw
        else:
            if(self.recursos.index<=2):
                sn = 1
            else:
                sn = -1
            ER = ecw

        #Verifica se o resultado ainda não está aceitável:
        if self.alinhado == False:
            go = (abs(ER) > self.const['RP'])

            if(go):
                #Correção brusca por limiar:
                if(abs(ER) > (self.const['LR'])):
                    vd = (-self.const['VROT']*sn)
                    ve = (self.const['VROT']*sn)
                else: #Ajuste proporcional final:
                    vd = (-self.const['KR']*ER*self.const['VROT']*sn)
                    ve = (self.const['KR']*ER*self.const['VROT']*sn)
            else: #Desliga os motores quando o alinhamento estiver satisfatório:
                vd = 0
                ve = 0
                self.alinhamento = True

        #Associa as velocidades aos motores:
        self.MTD.setVelocity(vd)
        self.MTE.setVelocity(ve)
        #Compara a tangente atual com o valor anterior para reativar o alinhamento se necessario
      
        
    def posicionardefesa(self,tillcon=False):
        #print("----------------------")
        #print("Posicionando para defesa")
        target = self.getTargetInfo("BOLA")
        modo = 1      
        #print(target)
        dx = (target[0]-self.recursos.this_pose[0])
        dy = (target[1]-self.recursos.this_pose[1])
        angle = (np.pi - np.arctan2(dy, dx))
        tg = [(target[0] - (self.recursos.const['offset']*np.cos(angle))), (target[1] + (self.recursos.const['offset']*np.sin(angle)))]
        dist = np.sqrt((dx**2)+(dy**2))
        #Obtendo o ângulo robô-alvo:
        
        #print(f"dx:{dx},dy:{dy}")
        #print(f"Angulo robo-alvo:{angle}, tangente: {tg}")
        #print("----------------------")
        
                   #Erro de rotação:
        #flags importantes para a decisao de estrategia
        #ballHold:ja tem a posse de bola, distancia do alvo:parametro auxiliar de velocidade
        #modos : seguir, alinhar, posicionar
        if tg != self.lastangle:
            self.alinhado = False
        print(tg)
        if modo == 1 and self.alinhado == False:
            theta = self.recursos.this_pose[2]
            print("theta:", theta)
            #Erro de rotação:
            eccw = abs(angle-theta)
            print("ECCW:",eccw)
            if(eccw>180):
                eccw = (360-eccw)
            ecw = (360-eccw)
            if(eccw<ecw):
                ER = eccw
                if(self.recursos.index<=2):
                    sn = -1
                else:
                    sn = 1
            else:
                ER = ecw
                if(self.recursos.index<=2):
                    sn = 1
                else:
                    sn = -1
            ET = (dist - self.recursos.const['offset'])
            
            print("Dist:", dist)
            #Define a condição de continuar:
            print("ET:",ET)
            print("BallHold:",self.recursos.ballHold())
            #modo 1 =  
            if(tillcon):
                go = (not self.recursos.ballHold())
                print(self.recursos.ballHold())
            else:
                go = (abs(ET) > self.recursos.const['TP'])
            #Verifica se o alinhamento e a posição ainda não estão aceitáveis:
            if abs(ET) < 130:
                go = False  # The robot is already at the desired position, so stop moving
                vd = 0
                ve = 0
            else:#Correção brusca por limiar:
                print(go)
                if(abs(ER) > self.recursos.const['LR']):
                    vd = (-self.recursos.const['VROT']*sn)
                    ve = (self.recursos.const['VROT']*sn)
                else: #Ajuste suave de rotação e posição simultâneas:
                    vd = (sat((1-np.sin(np.radians(ER*sn))), 1) * (sig(ET)*np.sqrt(abs(sat((ET*self.recursos.const['KT']), 1)))) * self.recursos.const['VMAX'])
                    ve = (sat((1+np.sin(np.radians(ER*sn))), 1) * (sig(ET)*np.sqrt(abs(sat((ET*self.recursos.const['KT']), 1)))) * self.recursos.const['VMAX'])
                    
        if go==False: #Desliga os motores quando o resultado estiver satisfatório:
            print("GO:",go)
            vd = 0
            ve = 0
            #Associa as velocidades aos motores:
        self.recursos.MTD.setVelocity(vd)
        self.recursos.MTE.setVelocity(ve)
        
        return go
        
        
        
        xyz= False
   
            

    def defesa(self):
    
        mode = 1        
        if(mode==1):
            if(not self.recursos.collide()):
                if(not self.recursos.gotoPos(self.recursos.ball, True)):
                    
                    mode += 1
            else:
                self.recursos.align(self.recursos.ball)
        if(mode==2):
            if(not self.recursos.collide()):
                if(not self.recursos.gotoPos([1000, 0])):
                    print(mode)
                    mode += 1
            else:
                self.recursos.align(self.recursos.ball)
        if(mode==3):
            if(not self.recursos.randomShoot()):
                    print(mode)
                    mode = 1
        
        
