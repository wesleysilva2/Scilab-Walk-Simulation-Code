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
myPosX = zeros(1,20); // Vou precisar definir uma posição e subitrair ela pelo finalTarget para obter a WalkOri
myPosY = zeros(1,20);

velocityX = zeros(1,20);
velocityY = zeros(1,20);

vX = zeros(1,20);
vy = zeros(1,20);

walkOri = 31.2234; // Valor Fixo provisoriamente

//inicio do calculo

for k = 1:20

//walkOri = (finalTargetX(k) - myPosX(k)) + (finalTargetY(k) - myPosY(k)); WalkOri começa com zero, ele recebe o valor que é o finalTarget - SuaPosição
//Variaveis Para o calculo da velocidade
maxVx = 1 * cos(walkOri);
minVx = -0.5 * cos(walkOri);
maxVy = 0.6 * sin(walkOri);
minVy = -0.6 * sin(walkOri);

[velocityX(k),velocityY(k)] = thetaToSpeed(maxVx,minVx,maxVy,minVy,walkOri);

vX(k) = k*(velocityX(k)); // Calculo da velocidade que vai para a função principal da caminhada
vY(k) = k*(-1 * velocityY(k));


end

// FOOT GENERATOR2
 
 //Parametros da função footgenerator2 com valores constantes  
 stepTheta = 0; 
 stepNumber = 8; // Esses valores são definidos no arquivo P0 da caminhada, no codigo é 7 botei 8 pq no Scilab começa com 7
 TimeStep = 0.136988;
 legSeparation = 0.11;
 legExtention = 0.200636;
 
 params_period = 0.136988; // Variaveis para o calculo do Dx e Dy
 params_dXmult = 0.952396; // Que calcula junto com os valores que vem do Agent Actions
 params_dYmult = 0.378298; // Esses valores são definidos no arquivo P0 da caminhada
 
 params_minDx =  -0.02951;
 params_minDy = -0.005424;
 params_maxDx = params_minDx*-1;
 params_maxDy = params_minDy*-1;
 
 dX = zeros(1,20); // No calculo normal dx, dy, inputdx e dy são doubles e não vetores
 dY = zeros(1,20); // Utilizei vetores aqui para simular varios calculos deles (ideia inicial, pode mudar)
 lastdX = zeros(1,20);
 lastdY = zeros(1,20);
 inputDX = zeros(1,20);
 inputDY = zeros(1,20);
 
 // PlanedFoot e InitFoot inicialização das variaveis
 
 PositionL = struct('X',0,'Y',0.055); // Posição inicial do foot em Right e Left
 PositionR = struct('X',0,'Y',-0.055);
 
 inicialLeftLeg = struct('Position',PositionL,'Suporte',%T,'Right',%F,'Time',0,'Theta',0); // %T e %F é True e False no Scilab
 inicialRightLeg = struct('Position',PositionR,'Suporte',%T,'Right',%F,'Time',0,'Theta',0); // 0.055 é o legSeparation / 2
  
  for g = 1:25 // Inicializando um vetor de 25 passos planejados, como no codigo
    planedLeftFoot(g) = struct('Position',PositionL,'Suporte',%T,'Right',%F,'Time',0,'Theta',0);
    planedRightFoot(g) = struct('Position',PositionR,'Suporte',%T,'Right',%F,'Time',0,'Theta',0);
  end


 inicialLeftLeg = planedLeftFoot(1);
 inicialRightLeg = planedRightFoot(1);
 
 for t = 1:10 // Para o calculo do FootGenearotor é dito que só é utilizado os primeiros 6 planejados
     
     Max_in_X = 0;
     Max_in_Y = 0;
     Min_in_X = 0;
     Min_in_Y = 0;
     
     
     
     dX(t) =  vX(t)*params_period*params_dXmult; // Calculo da direção feita na função setOptParameter que foi chamada no AgentActions
     dY(t) =  vY(t)*params_period*params_dXmult;
     
     //Emulando calculo da função setWalkParameter aqui, basicamente a função inputDX = lastDX+Max(Min(dX-lastDX,maxDx),minDx);
     
     [Min_in_X]=MIN((dX(t) - lastdX(t)),params_maxDx); // Preciso definir primeiro os valores maximos e minimos em Dx e Dy 
     
     [Max_in_X] = MAX(Min_in_X,params_minDx); // Defino comparando com o padrão estabelecido pelo arquivo Po
     
     [Min_in_Y]=MIN((dY(t) - lastdY(t)),params_maxDy); 
     
     [Max_in_Y] = MAX(Min_in_Y,params_minDy);
     
     inputDX(t) = lastdX(t) + Max_in_X; // Valor que vai para o footGenearator2
     
     inputDY(t) = lastdY(t) + Max_in_Y;
     
     
 end

//Calculo do Foot Generator

hL = struct('X',0,'Y',0.055); // Posição Das juntas do agente 
hR = struct('X',0,'Y',-0.055);
d = struct('stepX',inputDX,'stepY',inputDY)

x = struct('X',0,'Y',0);
com = struct('X',0,'Y',0);


if (planedLeftFoot(1).Suporte & planedRightFoot(1).Suporte) then
    x.X = planedLeftFoot(1).Position.X - hL.X
    x.Y = planedLeftFoot(1).Position.Y - hL.Y
    planedLeftFoot(1).Suporte = %F;
end

if (~planedLeftFoot(1).Suporte) then
    x.X = planedLeftFoot(1).Position.X - hL.X
    x.Y = planedLeftFoot(1).Position.Y - hL.Y
end

if (~planedRightFoot(1).Suporte) then
    x.X = planedRightFoot(1).Position.X - hL.X
    x.Y = planedRightFoot(1).Position.Y - hL.Y
end


 for z = 2:stepNumber // Coloquei 2 para iniciar o for por que na função do foot ele coloca z - 1 na posição
     
     com.X = d.stepX + x.X;
     com.Y = d.stepY + x.Y;
     x.X=((com.X - x.X)*2) + x.X; // Com a posição dos pe de suporte x calculada podemos calcular uma projeção do proximo passo
     x.Y=((com.Y - x.Y)*2)+ x.Y;
     
    if (planedLeftFoot(z-1).Suporte) then
        newTheta = planedLeftFoot(z-1).Theta + stepTheta;
        pos = struct('X',0,'Y',0); // Nova posição a ser calculada
        pos.X = x.X + hL.X; 
        pos.Y = x.Y + hL.Y;
        planedLeftFoot(z-1) = struct('Position',pos,'Suporte',%F,'Right',planedLeftFoot(z-1).Right,'Time',planedLeftFoot(z-1).Time+TimeStep,'        Theta',newTheta)
    else 
        newTheta = planedLeftFoot(z-1).Theta + stepTheta;
        pos = struct('X',0,'Y',0); 
        pos.X = planedLeftFoot(z-1).Position.X;
        pos.Y = planedLeftFoot(z-1).Position.Y;
        planedLeftFoot(z-1) = struct('Position',pos,'Suporte',%T,'Right',planedLeftFoot(z-1).Right,'Time',planedLeftFoot(z-1).Time+TimeStep,'        Theta',newTheta)
    end
    
    if (planedRightFoot(z-1).Suporte) then
        newTheta = planedRightFoot(z-1).Theta + stepTheta;
        pos = struct('X',0,'Y',0); // Nova posição a ser calculada
        pos.X = x.X + hR.X; 
        pos.Y = x.Y + hR.Y;
        planedRightFoot(z-1) = struct('Position',pos,'Suporte',%F,'Right',planedRightFoot(z-1).Right,'Time',planedRightFoot(z-1).Time+TimeStep,'        Theta',newTheta);
    else
        newTheta = planedRightFoot(z-1).Theta + stepTheta;
        pos = struct('X',0,'Y',0); 
        pos.X = planedRightFoot(z-1).Position.X; 
        pos.Y = planedRightFoot(z-1).Position.Y; 
        planedRightFoot(z-1) = struct('Position',pos,'Suporte',%T,'Right',planedRightFoot(z-1).Right,'Time',planedRightFoot(z-1).Time+TimeStep,'        Theta',newTheta)    
    end
    
    
    //Ajustes nas posições calculadas de acordo com os limites pré definidos, definindo o tamanho do passo
    
    minLegSperationY=0.08;//0.055;
    maxLegSeperationX=legExtention;
    maxLegSeperationY=legExtention; // Valores importantes para que o agente saiba até onde ele pode levar o pé, seus limites fisicos
    
    if (planedRightFoot(z).Suporte) then
        leftToRight = struct('X',0,'Y',0);
        leftToRight.X = planedRightFoot(z).Position.X - planedLeftFoot(z).Position.X;
        leftToRight.Y = planedRightFoot(z).Position.Y - planedLeftFoot(z).Position.Y;
        
        [MaxX] = MAX(leftToRight.X,-maxLegSeperationX); //Aplicando os limistes do agente 
        [leftToRight.X] = MIN(MaxX,maxLegSeperationX);
        [MaxY] = MAX(leftToRight.Y,-maxLegSeperationY);
        [leftToRight.Y] = MIN(MaxY,minLegSperationY);
        
        planedRightFoot(z).Position.X = planedLeftFoot(z).Position.X + leftToRight.X;
        planedRightFoot(z).Position.Y = planedLeftFoot(z).Position.Y + leftToRight.Y;
        
    end
    
    if (planedLeftFoot(z).Suporte) then
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
    
 end
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
