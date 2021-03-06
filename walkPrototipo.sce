clear
clc

//Função para o calculo da velocidade, função replicada do codigo thetaToSpeed
function [vx,vy] = thetaToSpeed(maxVx,minVx,maxVy,minVy,theta)
    
    if(theta >=0 & theta <=90) then
        if(tan(theta) <= (maxVy/maxVx)) then
            vx = maxVx;
            vy = tan(theta) * maxVx;
        else
            vx = maxVy/tan(theta);
            vy = maxVy
        end
    end
    
    if(theta > 90 & theta <=180) then
        if(tan(theta) <= (maxVy/minVx)) then
            vy = maxVy;
            vx = maxVy/tan(theta);
        else
            vx = minVx
            vy = minVx * tan(theta)
        end
    end
    
    if(theta < 0 & theta >= -90) then
        if(tan(theta) <= (minVy/maxVx)) then
            vy = minVy;
            vx = minVy/tan(theta);
        else
            vx = maxVx
            vy = maxVx * tan(theta)
        end
    end
    
    if(theta < -90 & theta > -180) then
        if(tan(theta) <= (minVy/minVx)) then
            vx = minVx;
            vy = tan(theta)*minVx;
        else
            vx = minVy/tan(theta);
            vy = minVy;
        end
    end

endfunction

function [A] = MAX(val1,val2)
    
    if (val1 > val2) then
        A = val1;
        
    else
        A = val2;
    end
endfunction

function [A] = MIN(val1,val2)
    
    if (val1 < val2) then
        A = val1;
        
    else
        A = val2;
    end
endfunction


// Simulação do AGENTACTIONS

//Declaração de variaveis vetores todos preenchidos com zero
finalTargetX = zeros(1,20); //Vetores para definir o ponto, ele geralmente so usa os 6 proximos no calculo do footgenerator2
finalTargetY = zeros(1,20); 
direction = zeros(1,20); // É o dir do codigo, mudei o nome aqui por ter uma palavra reservada dir no Scilab
k = 0.2;

velocityX = zeros(1,20);
velocityY = zeros(1,20);

vX = zeros(1,20);
vy = zeros(1,20);

walkOri = 31.2234; // Valor Fixo provisoriamente -0.92563

//inicio do calculo

for b = 1:20

//walkOri = (finalTargetX(k) - myPosX(k)) + (finalTargetY(k) - myPosY(k)); WalkOri começa com zero, ele recebe o valor que é o finalTarget - SuaPosição
//Variaveis Para o calculo da velocidade
maxVx = 1 * cos(walkOri);
minVx = -0.5 * cos(walkOri);
maxVy = 0.6 * sin(walkOri);
minVy = -0.6 * sin(walkOri);

[velocityX(b),velocityY(b)] = thetaToSpeed(maxVx,minVx,maxVy,minVy,walkOri);

vX(b) = k*(velocityX(b)); // Calculo da velocidade que vai para a função principal da caminhada
vY(b) = k*(-1 * velocityY(b));


end

// FOOT GENERATOR2
 
 //Parametros da função footgenerator2 com valores constantes  
 stepTheta = 0; 
 stepNumber = 8; // Esses valores são definidos no arquivo P0 da caminhada, no codigo é 6 botei 7 pq no Scilab começa com 1
 stepTime = 0.136988; // O setWalkParameters altera o valor de 0.8 para o valor do aquivo P0
 legSeparation = 0.11;
 legExtention = 0.200636;
 
 params_period = 0.136988; // Variaveis para o calculo do Dx e Dy
 params_dXmult = 0.952396; // Que calcula junto com os valores que vem do Agent Actions
 params_dYmult = 0.378298; // Esses valores são definidos no arquivo P0 da caminhada
 
 params_minDx =  -0.02951;
 params_minDy = -0.005424;
 params_maxDx = params_minDx*-1;
 params_maxDy = params_minDy*-1;
 deltaT = 0.002;
 
 dX = 0; // No calculo normal dx, dy, inputdx e dy são doubles e não vetores
 dY = 0; // Utilizei vetores aqui para simular varios calculos deles (ideia inicial, pode mudar)
 lastdX = 0;
 lastdY = 0;
 inputDX = 0;
 inputDY = 0;
 
 // PlanedFoot e InitFoot inicialização das variaveis
 
 PositionL = struct('X',0,'Y',0); // Posição inicial do foot em Right e Left
 PositionR = struct('X',0,'Y',0);
 
 InitPositionL = struct('X',0,'Y',0.055); // Posição inicial do foot em Right e Left
 InitPositionR = struct('X',0,'Y',-0.055);
 
 inicialLeftLeg = struct('Position',InitPositionL,'Suporte',%T,'Right',%F,'Time',0,'Theta',0); // %T e %F é True e False no Scilab
 inicialRightLeg = struct('Position',InitPositionR,'Suporte',%T,'Right',%F,'Time',0,'Theta',0); // 0.055 é o legSeparation / 2
  
  for g = 1:25 // Inicializando um vetor de 25 passos planejados, como no codigo
    planedLeftFoot(g) = struct('Position',PositionL,'Suporte',%T,'Right',%F,'Time',0,'Theta',0);
    planedRightFoot(g) = struct('Position',PositionR,'Suporte',%T,'Right',%F,'Time',0,'Theta',0);
  end

  planedLeftFoot(1) = inicialLeftLeg;
  planedRightFoot(1) = inicialRightLeg;
  planedLeftFoot(2) = inicialLeftLeg;
  planedRightFoot(2) = inicialRightLeg;
 
 
// Para o calculo do FootGenearotor é dito que só é utilizado os primeiros 6 planejados 

// setWalkParameter
     
     Max_in_X = 0;
     Max_in_Y = 0;
     Min_in_X = 0;
     Min_in_Y = 0;
     
     dX =  vX(1)*params_period*params_dXmult; // Calculo da direção feita na função setOptParameter que foi chamada no AgentActions
     dY =  vY(1)*params_period*params_dXmult;
     
     //Emulando calculo da função setWalkParameter aqui, basicamente a função inputDX = lastDX+Max(Min(dX-lastDX,maxDx),minDx);
     
      sub1 = dX - lastdX;
     
     [Min_in_X]=MIN(sub1,params_maxDx); // Preciso definir primeiro os valores maximos e minimos em Dx e Dy 
     
     [Max_in_X] = MAX(Min_in_X,params_minDx); // Defino comparando com o padrão estabelecido pelo arquivo Po
     
      sub2 = dY - lastdY;
     
     [Min_in_Y]=MIN(sub2,params_maxDy); 
     
     [Max_in_Y] = MAX(Min_in_Y,params_minDy);
          
     inputDX = lastdX + Max_in_X; // Valor que vai para o footGenearator2
     
     inputDY = lastdY + Max_in_Y; 

     
//Calculo do Foot Generator, Falta só a questão do rotate

hL = struct('X',0,'Y',0.055); // Posição Das juntas do agente 
hR = struct('X',0,'Y',-0.055);
d = struct('stepX',inputDX,'stepY',inputDY);

x = struct('X',0,'Y',0);
com = struct('X',0,'Y',0);
newTheta = 0; 


if (planedLeftFoot(1).Suporte == %T & planedRightFoot(1).Suporte == %T) then
    x.X = planedLeftFoot(1).Position.X - hL.X;
    x.Y = planedLeftFoot(1).Position.Y - hL.Y;
    planedLeftFoot(1).Suporte = %F;
end

if (~planedLeftFoot(1).Suporte) then
    x.X = planedLeftFoot(1).Position.X - hL.X;
    x.Y = planedLeftFoot(1).Position.Y - hL.Y;
end

if (~planedRightFoot(1).Suporte) then
    x.X = planedRightFoot(1).Position.X - hL.X;
    x.Y = planedRightFoot(1).Position.Y - hL.Y;
end


 for z = 2:stepNumber // Coloquei 2 para iniciar o for por que na função do foot ele coloca z - 1 na posição
     
     com.X = d.stepX + x.X;
     com.Y = d.stepY + x.Y;
     x.X=((com.X - x.X)*2) + x.X; // Com a posição dos pe de suporte x calculada podemos calcular uma projeção do proximo passo
     x.Y=((com.Y - x.Y)*2)+ x.Y;
     
    if (planedLeftFoot(z-1).Suporte == %T) then
        newTheta = planedLeftFoot(z-1).Theta + stepTheta;
        pos = struct('X',0,'Y',0); // Nova posição a ser calculada
        pos.X = x.X + hL.X; 
        pos.Y = x.Y + hL.Y;
        planedLeftFoot(z) = struct('Position',pos,'Suporte',%F,'Right',planedLeftFoot(z-1).Right,'Time',planedLeftFoot(z-1).Time+stepTime,'Theta',newTheta);
    else 
        newTheta = planedLeftFoot(z-1).Theta + stepTheta;
        pos = struct('X',0,'Y',0); 
        pos.X = planedLeftFoot(z-1).Position.X;
        pos.Y = planedLeftFoot(z-1).Position.Y;
        planedLeftFoot(z) = struct('Position',pos,'Suporte',%T,'Right',planedLeftFoot(z-1).Right,'Time',planedLeftFoot(z-1).Time+stepTime,'Theta',newTheta);
    end
    
    if (planedRightFoot(z-1).Suporte == %T) then
        newTheta = planedRightFoot(z-1).Theta + stepTheta;
        pos = struct('X',0,'Y',0); // Nova posição a ser calculada
        pos.X = x.X + hR.X; 
        pos.Y = x.Y + hR.Y;
        planedRightFoot(z) = struct('Position',pos,'Suporte',%F,'Right',planedRightFoot(z-1).Right,'Time',planedRightFoot(z-1).Time+stepTime,'Theta',newTheta);
    else
        newTheta = planedRightFoot(z-1).Theta + stepTheta;
        pos = struct('X',0,'Y',0); 
        pos.X = planedRightFoot(z-1).Position.X; 
        pos.Y = planedRightFoot(z-1).Position.Y; 
        planedRightFoot(z) = struct('Position',pos,'Suporte',%T,'Right',planedRightFoot(z-1).Right,'Time',planedRightFoot(z-1).Time+stepTime,'Theta',newTheta);   
    end
    
    
    //Ajustes nas posições calculadas de acordo com os limites pré definidos, definindo o tamanho do passo
    
    minLegSperationY=0.08;//0.055;
    maxLegSeperationX = legExtention;
    maxLegSeperationY = legExtention; // Valores importantes para que o agente saiba até onde ele pode levar o pé, seus limites fisicos
    
    if (planedRightFoot(z).Suporte == %F) then
          leftToRight = struct('X',0,'Y',0);
          leftToRight.X = planedRightFoot(z).Position.X - planedLeftFoot(z).Position.X;
          leftToRight.Y = planedRightFoot(z).Position.Y - planedLeftFoot(z).Position.Y;
            
          [MaxX] = MAX(leftToRight.X,-maxLegSeperationX); //Aplicando os limistes do agente 
          [leftToRight.X] = MIN(MaxX,maxLegSeperationX);
          [MaxY] = MAX(leftToRight.Y,-maxLegSeperationY);
          [leftToRight.Y] = MIN(MaxY,-minLegSperationY);
            
          planedRightFoot(z).Position.X = planedLeftFoot(z).Position.X + leftToRight.X;
          planedRightFoot(z).Position.Y = planedLeftFoot(z).Position.Y + leftToRight.Y;
            
    end
    
    if (planedLeftFoot(z).Suporte == %F) then
         rightToLeft = struct('X',0,'Y',0);
         rightToLeft.X = planedLeftFoot(z).Position.X - planedRightFoot(z).Position.X;
         rightToLeft.Y = planedLeftFoot(z).Position.Y - planedRightFoot(z).Position.Y;
            
        [MaxX] = MAX(rightToLeft.X,-maxLegSeperationX); //Aplicando os limistes do agente 
        [rightToLeft.X] = MIN(MaxX,maxLegSeperationX);
        [MaxY] = MAX(rightToLeft.Y,minLegSperationY);
        [rightToLeft.Y] = MIN(MaxY,maxLegSeperationY);
            
        planedLeftFoot(z).Position.X = planedRightFoot(z).Position.X + rightToLeft.X;
        planedLeftFoot(z).Position.Y = planedRightFoot(z).Position.Y + rightToLeft.Y;
            
    end
    
    lastdX = inputDX;
    lastdY = inputDY;
 end
 
 
 
 // ZMPGENERATOR
 
 timeStep = planedRightFoot(2).Time - planedRightFoot(1).Time; 
 initSize = 547 //antes era int(((stepNumber+1)*(timeStep))/deltaT);
 zmpPosition = struct('X',0,'Y',0);
 zmpValue = initSize+1000; //ZMP *zmp= new ZMP[initSize+1000]; Era initSize+1000
 
 for g = 1:zmpValue
   zmp(g) = struct('Position',zmpPosition,'Time',0); 
 end
 
 //zmp = zeros(1,initSize+1000); // zeros(1,20);
 sizeZ = 0;
 sizeStep = 0;
 
// Gerando valores no ZMP que se mantem constante a cada ciclo de 68 posições que é igual ao valor de SizeStep
  for a = 1:stepNumber
      if (planedRightFoot(a).Suporte == %T & planedLeftFoot(a).Suporte == %T) then
          sizeStep = int(timeStep/deltaT);
          for i = 1:sizeStep
            zmp((a*sizeStep)+i).Position.X = (planedRightFoot(a).Position.X + planedLeftFoot(a).Position.X)/2;
            zmp((a*sizeStep)+i).Position.Y = 0;
            zmp((a*sizeStep)+i).Time = (a*timeStep)+(i*deltaT);
            sizeZ = sizeZ + 1;
          end
      
      elseif(planedRightFoot(a).Suporte == %T) then 
          sizeStep = int(timeStep/deltaT);
          for i = 1:sizeStep
            zmp((a*sizeStep)+i).Position.X = planedRightFoot(a).Position.X;
            zmp((a*sizeStep)+i).Position.Y = planedRightFoot(a).Position.Y;
            zmp((a*sizeStep)+i).Time = (a*timeStep)+(i*deltaT);
            sizeZ = sizeZ + 1;
          end
          
      elseif(planedLeftFoot(a).Suporte == %T) then 
          sizeStep = int(timeStep/deltaT);
          for i = 1:sizeStep
            zmp((a*sizeStep)+i).Position.X = planedLeftFoot(a).Position.X;
            zmp((a*sizeStep)+i).Position.Y = planedLeftFoot(a).Position.Y;
            zmp((a*sizeStep)+i).Time = (a*timeStep)+(i*deltaT);
            sizeZ = sizeZ + 1;
          end
      end
           
  end  

// Calculo do CoM Vertical verticalCoMTrajectoryFW

B0 = 0.187;  // Valores de Beta, esses valores devem ser treinados para se obter os melhores 
B1 = -0.0025;  // Para a caminhada, Eles são os valores de amplitude da trajetoria da altura do quadril 
B2 =  0.0008; 

M_PI = 3.14159265358979323846;

forierFastWalk = 0; 

forierFastWalk = B0 + B1 * cos(2 * M_PI/stepTime) + B2 * cos(4 * M_PI/stepTime);


// Calculo do CoM Horizontal

 
 for f = 1:stepNumber // Plotar os passos planejados
    
      
     plot(f,planedRightFoot(f).Position.X,"mo");
     plot(f,planedRightFoot(f).Position.Y,"go");
     plot(f,planedLeftFoot(f).Position.X,"ro");
     plot(f,planedLeftFoot(f).Position.Y,"ko");
     
      
      for i = 1:sizeStep 
          plot(f,zmp((f*sizeStep)+i).Position.X,"o");
          plot(f,zmp((f*sizeStep)+i).Position.Y,"co");
          hl=legend(['Rx';'Ry';'Lx';'Ly';'zmpX';'zmpY'],"out_upper_right");
          //hl=legend(['Rx';'Ry';'zmpX';'zmpY'],"out_upper_right");
      end
     
     
     //plot(f,planedRightFoot(f).Position.Y,"ro");
     //plot(f,planedRightFoot(f).Position.X,"ro");
     //plot(f,planedLeftFoot(f).Position.Y,"ro");
     //plot(f,planedLeftFoot(f).Position.X,"o");
     
        
     ylabel("Valor da Posição do Passo");
     xlabel("Numero de Passos");
     
     //plot(zmp(f).Position.X, "o");
 end
 
 
 
