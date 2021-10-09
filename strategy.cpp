#include "strategy.h"


Strategy::Strategy(bool time)
{
    L = 0.04; //Distância entre roda e centro
    R = 0.02; //Raio da roda

    vector<double> aux{0,0};

    for(int i = 0; i < 5 ; i++){
       velocidades[i] = 0;
    }

    qtdRobos = 3;
    vrMax = 200; // era 125
    Vmax = 2.5; // era 2.5
    Wmax = 62.5;

    for(int i = 0; i < qtdRobos; i++)
    {
        vRL.push_back(aux);
        VW.push_back(aux);
    }

    //Inicialização do vetor de preditor
    int N = 5;
    ballPredPos temp;
    temp.x = 0;
    temp.y = 0;

    for(int i = 0; i < N; i++)
    {
        ballPredMemory.push_back(temp);
    }

    predictedBall = temp;

    //Inicialização dos vetores de memória PID
    int N_mem = 5;

    for(int i=0; i < N_mem; i++)
    {
        memoria_azul_linear.push_back(0);
        memoria_azul_angular.push_back(0);
    }

    //verifica se foi criado um time amarelo
    if(time == true){
        lado = -1;
        cout << "AMARELO" << endl;
    }else{//ou time azul
        lado = 1;
        cout << "AZUL" << endl;
    }

    replain = true; //Modo planejar
    plain = false;
    flagTbT = true;
    Mdist=0;

    tempo = 0;

}

//Está fazendo muito pouca diferença, talvez deva diminuir o tempo
//de amostragem.
void Strategy::predict_ball(fira_message::Ball ball)
{

    // Depende de quantas foram inicializadas no construtor
    // Retornarei um vetor de 3 pontos, tais que os primeiros elementos são futuros mais próximos
    int N = 0;
    float a = 0;     // Começo da pseudoinversão (A'A)^(-1)
    float theta = 0; //Guarda o termo do preditor
    float temp = 0;

    int futureTime = 50; //Representa quantas iterações no futuro o robô vai predizer a bola
    //Atenção: quanto mais no futuro, mais errado, então imagine esse valor como um ganho de preditor
    //tal que MAIOR = mais pra frente, MENOR = menos pra frente no tempo.

    N = this->ballPredMemory.size()-1; // indice do ultimo elemento

    ballPredPos result; //Resultado será guardado aqui.

    //Atualização do vetor de memória
    ballPredPos ballUpdate;
    ballUpdate.x = ball.x();
    ballUpdate.y = ball.y();
    ballPredMemory.push_back(ballUpdate); //Adicionando novo elemento à memória
    ballPredMemory.erase(ballPredMemory.begin()+0); //Removendo último valor da memória

    //Primeiro para a posição x.
    for (int m = 0; m < (N-1);m++)
       a = a + pow(this->ballPredMemory[m].x,2);
    a = 1/(a + 0.001);

    for(int m = 0; m < (N-1);m++)
       theta = theta + a * this->ballPredMemory[m].x * this->ballPredMemory[m+1].x;
    theta = pow(theta,futureTime);
    result.x = theta * ball.x();

    //Agora pra posicao y.
    a = 0;
    theta = 0;

    for (int m = 0; m < (N-1);m++)
       a = a + pow(this->ballPredMemory[m].y,2);
    a = 1/(a + 0.001);

    for(int m = 0; m < (N-1);m++)
       theta = theta + a * this->ballPredMemory[m].y * this->ballPredMemory[m+1].y;
    theta = pow(theta,futureTime);
    result.y = theta * ball.y();

    //"Retorno" da função.
    this->predictedBall = result;
}

void Strategy::strategy_blue(fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                             fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2,
                             fira_message::Ball ball, const fira_message::Field & field,string sit_juiz)
{
   //robots = Team(b0,b1,b2,y0,y1,y2);
   blue = Team(b0,b1,b2);
   yellow = Team(y0,y1,y2);

   double dist1, dist2;
   cout << endl;
   if (ball.x()<0.65)
    {
        dist1 = sqrt(pow(b1.x()-ball.x(),2)+pow(b1.y()-ball.y(),2));
        dist2 = sqrt(pow(b2.x()-ball.x(),2)+pow(b2.y()-ball.y(),2));
    }
    else //Se a bola estiver na banheira, a distância >y< importa mais
    {
        dist1 = sqrt(pow(b1.x()-ball.x(),2)+4*pow(b1.y()-ball.y(),2));
        dist2 = sqrt(pow(b2.x()-ball.x(),2)+4*pow(b2.y()-ball.y(),2));
    }
    if(1 /*sit_juiz == "GAME_ON"*/){
         goleiro_petersson2(b0,ball,0);
         if((ball.x() > -0.1)&&((dist1>0.15)||(dist2>0.15))){
             if (dist1 > dist2){
                 cout<<"config 1"<<endl;
                 zagueiro2(b1,ball,1);
                 atacante_todos(blue,yellow,ball,2,1);
             }else{
                 cout<<"config 2"<<endl;
                 zagueiro2(b2,ball,2);
                 atacante_todos(blue,yellow,ball,1,2);
             }
         }else{
             zagueiro2(b1,ball,1);
             atacante_todos(blue,yellow,ball,2,1);
         }
    }else{
        andarFrente(0,0);
        andarFrente(0,1);
        andarFrente(0,2);
    }

    //goleiro_petersson2(b0,ball,0);
    //zagueiro2(b1,ball,1);
    //atacante_todos(blue,yellow,ball,2,1);

    /*if(replain)
        {
            caminho = gerar_caminho(20);
            replain=false;
            detectChange = 0;
        }*/

    //caminho.push_back(make_pair(ball.x(),ball.y()));

   //Curr_vel = Velocidade atual do robô 1
   /*double Curr_velR1 = sqrt(pow(b1.vx(), 2)+pow(b1.vy(), 2));
   double l_ahead=0.07;   //Look-Ahead distance
   double raio_rrt=0.45;  //Raio da RRT
   double a = 0.85; //frequência de corte
   pair<double,double> goalP=make_pair(ball.x(),ball.y());
   pair<double,double> currPos=make_pair(b1.x(),b1.y());
   vector<vetor> caminho_aux;
   if(replain)
       {
            start=clock();
           //caminho=takeBallToGoal(make_pair(b1.x(),b1.y()),make_pair(ball.x(),ball.y()));
           //caminho=gerar_caminho(100);
             caminho.clear();
             while (!(caminho.size()>1))
                {
                   if(plain && (Curr_velR1>0.01))
                      currPos=carrot_point;
                   caminho= path_planningRRT(currPos,goalP,robots,1,100,raio_rrt);
                   filtro_caminho(a);
                   setup_pure_pursuit(caminho.at(0));
                }
             plain = true;
             replain=false;
             end = clock();
             tempo=pow(10,3)*(end-start)/CLOCKS_PER_SEC;
             cout << "Tempo de Planejamento: " << tempo << "ms "<<endl;
       }
   //critério do produto escalar (repleneja se a bola estiver muito desvida do objetivo)
   bool cri = dotCriterio(currPos,carrot_point,goalP);
   if((Curr_velR1<0.01)||(cri==true))
       replain=true;
   double v_pref = 0.5;
   pure_pursuit(b1,1,caminho,l_ahead,v_pref);*/
   //pair<double,double> bola=make_pair(ball.x(),ball.y());
   //pair<double,double> goalP=make_pair(-0.76,0);
   //vaiParaRRT(b1,1,goalP,bola);
   //takeBallToGoal(b1,1,goalP);


   //vaiPara(b0,ball.x()-0.1,ball.y()-0.1,0);

    cinematica_azul();

    //export data
    send_data_robots(robots);
    send_data_ball(carrot_point.first,carrot_point.second);
    send_data_path(caminho);
    //send_data_control(v_pref,Curr_velR1);




}

void Strategy::strategy_yellow(fira_message::Robot y0, fira_message::Robot y1,fira_message::Robot y2,
                               fira_message::Robot b0, fira_message::Robot b1,fira_message::Robot b2,
                               fira_message::Ball ball, const fira_message::Field & field,string sit_juiz)
{
    Team blue(b0,b1,b2);
    Team yellow(y0,y1,y2);
    double dist1, dist2;

    if (ball.x()>-0.65)

    {
        dist1 = sqrt(pow(y1.x()-ball.x(),2)+pow(y1.y()-ball.y(),2));
        dist2 = sqrt(pow(y2.x()-ball.x(),2)+pow(y2.y()-ball.y(),2));
    }
    else //Se a bola estiver na banheira, a distância >y< importa mais
    {
        dist1 = sqrt(pow(y1.x()-ball.x(),2)+4*pow(y1.y()-ball.y(),2));
        dist2 = sqrt(pow(y2.x()-ball.x(),2)+4*pow(y2.y()-ball.y(),2));
    }

    if(/*sit_juiz == "GAME_ON"*/ 1){

        goleiro_petersson2(y0,ball,0);
         if((ball.x() < -0.1)&&((dist1>0.15)||(dist2>0.15))){
             if (dist1 > dist2){
                 zagueiro2(y1,ball,1);
                 atacante_todos(yellow,blue,ball,2,1);
             }else{
                 zagueiro2(y2,ball,2);
                 atacante_todos(yellow,blue,ball,1,2);
             }
         }else{
             zagueiro2(y1,ball,1);
             atacante_todos(yellow,blue,ball,2,1);
         }

    }else{
        andarFrente(0,0);
        andarFrente(0,1);
        andarFrente(0,2);
    }

    cinematica_amarelo();
}

//Calcula as velocidades a serem enviadas ao robô, utilizando cinematica inversa
void Strategy::cinematica_azul()
{
    double k[] = {1,0.6,0.6};

    for(int i = 0; i < qtdRobos; i++)
    {
        vRL[i][0] = (VW[i][0]+VW[i][1]*L)/R;
        vRL[i][1] = (VW[i][0]-VW[i][1]*L)/R;

        vRL[i][0] = limita_velocidade(vRL[i][0],vrMax);
        vRL[i][1] = limita_velocidade(vRL[i][1],vrMax);

        vRL[i][0] = velocidades_azul[i][0]*(1 - k[i]) + vRL[i][0]*k[i];
        vRL[i][1] = velocidades_azul[i][1]*(1 - k[i]) + vRL[i][1]*k[i];

        velocidades_azul[i][0] = vRL[i][0]*k[i];
        velocidades_azul[i][1] = vRL[i][1]*k[i];

        if(bandeira) //Bandeira relacionada ao Firekick
        {//Se for fire kick, ela é falsa
            vRL[i][0] = velocidades_azul[i][0]*(1 - k[i]) + vRL[i][0]*k[i];
            vRL[i][1] = velocidades_azul[i][1]*(1 - k[i]) + vRL[i][1]*k[i];

            velocidades_azul[i][0] = vRL[i][0]*k[i];
            velocidades_azul[i][1] = vRL[i][1]*k[i];
        }

    }

}

void Strategy::andarFrente(double vel, int id)
{
    double Vaux = vel/vrMax;
    VW[id][0] = Vmax*Vaux;
    VW[id][1] = 0;
}
void Strategy::cinematica_amarelo()
{
    double k[] = {1,0.6,0.6};
    for(int i = 0; i < qtdRobos; i++)
    {
        vRL[i][0] = (VW[i][0]+VW[i][1]*L)/R;
        vRL[i][1] = (VW[i][0]-VW[i][1]*L)/R;

        vRL[i][0] = limita_velocidade(vRL[i][0],vrMax);
        vRL[i][1] = limita_velocidade(vRL[i][1],vrMax);

        if(bandeira)
        {
            vRL[i][0] = velocidades_amarelo[i][0]*(1 - k[i]) + vRL[i][0]*k[i];
            vRL[i][1] = velocidades_amarelo[i][1]*(1 - k[i]) + vRL[i][1]*k[i];

            velocidades_amarelo[i][0] = vRL[i][0]*k[i];
            velocidades_amarelo[i][1] = vRL[i][1]*k[i];
        }
    }
}

void Strategy::andarFundo(double vel, int id)
{
    double Vaux = vel/vrMax;
    VW[id][0] = -Vmax*Vaux;
    VW[id][1] = 0;
}

void Strategy::girarHorario(double vel,int id)
{
    double Waux = vel/vrMax;
    VW[id][0] = 0;
    VW[id][1] = Wmax*Waux;
}

void Strategy::girarAntihorario(double vel,int id)
{
    double Waux = vel/vrMax;
    VW[id][0] = 0;
    VW[id][1] = -Waux*Wmax;
}

void Strategy::vaiPara(fira_message::Robot rb, double px, double py, int id)
{
    VW[id][0] = controleLinear(rb,px,py,0);
    ang_err angulo = olhar(rb, px, py);
    VW[id][1] = controleAngular(angulo.fi);
}

void Strategy::vaiParaDinamico(fira_message::Robot rb, double px, double py, int id)
{
    ang_err angulo = olhar(rb, px, py);
    double erro_angular = angulo.fi; //de -180 a 180
    double erro_linear = angulo.flag*distancia(rb,px,py);


    double V = 0;
    double W = 0;

    double Kp_l = 10;
    double Ki_l = 5;
    double Kd_l = 0.01;

    double Kp_a = 0.15;
    double Ki_a = 0.02;
    double Kd_a = 0.01;

    double temp_integral_l = 0;
    double temp_integral_a = 0;

    if(lado == 1){
        for (int i = 0; i < (int)memoria_azul_linear.size(); i++)
        {
            temp_integral_l = temp_integral_l + memoria_azul_linear.at(i);
        }

        for (int i = 0; i < (int)memoria_azul_angular.size(); i++)
        {
            temp_integral_a = temp_integral_a + memoria_azul_angular.at(i);
        }

        V = Kp_l*erro_linear + Ki_l*temp_integral_l + Kd_l*(erro_linear-memoria_azul_linear.at(0));

        W = Kp_a*erro_angular + Ki_a*temp_integral_a + Kd_a*(erro_angular-memoria_azul_angular.at(0));

        VW[id][0] = V;
        VW[id][1] = W;
        atualiza_memoria_azul(erro_linear,erro_angular);
    }else{
        for (int i = 0; i < (int)memoria_amarelo_linear.size(); i++)
        {
            temp_integral_l = temp_integral_l + memoria_amarelo_linear.at(i);
        }

        for (int i = 0; i < (int)memoria_amarelo_angular.size(); i++)
        {
            temp_integral_a = temp_integral_a + memoria_amarelo_angular.at(i);
        }

        V = Kp_l*erro_linear + Ki_l*temp_integral_l + Kd_l*(erro_linear-memoria_amarelo_linear.at(0));

        W = Kp_a*erro_angular + Ki_a*temp_integral_a + Kd_a*(erro_angular-memoria_amarelo_angular.at(0));

        VW[id][0] = V;
        VW[id][1] = W;
        atualiza_memoria_amarelo(erro_linear,erro_angular);
    }
}

void Strategy::atualiza_memoria_azul(double linear, double angular)
{
    //Removendo últimos elementos
    memoria_azul_linear.pop_back();
    memoria_azul_angular.pop_back();

    //Inserindo novos elementos no começo
    memoria_azul_linear.insert(memoria_azul_linear.begin(),linear);
    memoria_azul_angular.insert(memoria_azul_angular.begin(),angular);
}

void Strategy::atualiza_memoria_amarelo(double linear, double angular)
{
    //Removendo últimos elementos
    memoria_amarelo_linear.pop_back();
    memoria_amarelo_angular.pop_back();

    //Inserindo novos elementos no começo
    memoria_amarelo_linear.insert(memoria_amarelo_linear.begin(),linear);
    memoria_amarelo_angular.insert(memoria_amarelo_angular.begin(),angular);
}


double Strategy::controleAngular(double fi2) // função testada. lembrete : (sinal de w) = -(sinal de fi)
{
    //double W_max = -0.3;    // constante limitante da tangente hiperbólica : deve ser negativa
    double W_max = Wmax;
    double k_ang = 0.2;
    double Waux = 0;
    double fi = fi2/90; // coloca o erro entre -1 e 1

    Waux = W_max*tanh(k_ang*fi); // nгo linear
                              //  W =  kw*fi;                     // proporcional

    Waux = limita_velocidade(Waux, Wmax); //satura em -1 a 1

    return(Waux); //deve retornar um valor entre -1 e 1
}

double Strategy::controleLinear(fira_message::Robot rb,double px, double py,double l)
{
    double  Vaux = 0;
    double  k_lin = 4;   //constante de contração da tangente hiperbólica Rapha colocou 0.8
    double  V_max = Vmax;       //constante limitante da tangente hiperbólica
    double  v_min = 0.5;  	 //módulo da velocidade linear mínima permitida Rapha colocou 0.03
    double  ang_grande = 30; //para ângulos maiores que esse valor o sistema da prioridade ao W, reduzindo o V
    double  dist = distancia(rb, px, py)-l;

    ang_err angulo = olhar(rb, px, py);

    Vaux = V_max*tanh(k_lin*dist*angulo.flag);  //controle não linear de V

    //if (Vaux*angulo.flag < v_min) Vaux = v_min*angulo.flag;  //aplica o valor definido em v_min

    //if (angulo.fi*angulo.flag > ang_grande) V = v_min*angulo.flag;  // controle de prioridade reduzindo V quando "ang_err" for grande
    Vaux = Vaux*abs(cos(angulo.fi*M_PI/180)); // controle de prioridade reduzindo V quando "ang_err" for grande

    Vaux = limita_velocidade(Vaux, Vmax); //satura em -Vmax a Vmax

    return(Vaux);
}

ang_err Strategy::olhar(fira_message::Robot rb, double px, double py)   // função testada - ok!
{
      double r = rb.orientation()*180/M_PI; // orientação do robô de -180 a 180
      ang_err angulo;

      angulo.flag = 1;
      angulo.fi = atan2((py - rb.y()) , (px - rb.x()))*180/M_PI;  //ângulo entre -180 e 180

      if (r < 0)  {r = r + 360;}          //muda para 0 a 360
      if (angulo.fi < 0)   {angulo.fi = angulo.fi + 360;}      //muda para 0 a 360

      angulo.fi = angulo.fi - r;  //erro de orientação a ser corrigido

      if (angulo.fi > 180) {angulo.fi = angulo.fi - 360;}  // limita entre -180 e 180
      if (angulo.fi < -180) {angulo.fi = angulo.fi + 360;} // limita entre -180 e 180

      if (angulo.fi > 90) // se for mais fácil, olhar com o fundo...
      {
          angulo.fi = angulo.fi - 180;
          angulo.flag = -1;
      }
      if (angulo.fi < -90) // se for mais fácil, olhar com o fundo...
      {
          angulo.fi = 180 + angulo.fi;
          angulo.flag = -1;
      }

      // angulo.fi é o ângulo a ser corrigido na convenção do sistema de visão. valores entre -90 e 90.
      // angulo.flag é o sinal a ser aplicado na velocidade linear
      return(angulo);
}

double Strategy::distancia(fira_message::Robot rb, double px, double py)
{
      double dist = sqrt( pow((rb.x()-px),2) + pow((rb.y()-py),2) );
      return(dist);
}

double Strategy::distancia(double X, double Y, double px, double py)
{
    double dist = sqrt( pow((X-px),2) + pow((Y-py),2) );
    return(dist);
}

double Strategy::limita_velocidade(double valor, double sat)
{
      if (valor > sat) {valor = sat;}
      if (valor < -sat) {valor = -sat;}
      return(valor);
}

vector<pair<double, double>> Strategy::gerar_caminho(int qtd_pontos)
{
    /*vector<pair<double, double>> waypints = {make_pair(0,0),make_pair(-0.4,-0.3),
                                               make_pair(-0.4,0.3),make_pair(0.4,0.3),make_pair(0.4,-0.3)};*/
    vector<pair<double, double>> waypints;

   // double lado = 0.5;
    double raio = 0.46;
    for(int i = 0; i <qtd_pontos; i++)
        {
            double t = M_PI - 2*M_PI*((qtd_pontos-1)-i)/(qtd_pontos-1);
            //double x = 0.25*(1 + cos(t))*cos(t);
            //double y = 0.25*(1 + cos(t))*sin(t);
            double x = raio*cos(5*t);
            double y = raio*sin(5*t);
            waypints.push_back(make_pair(x,y));
           /* waypints.push_back(make_pair(-lado,-lado));
            waypints.push_back(make_pair(lado,-lado));
            waypints.push_back(make_pair(-lado,lado));
            waypints.push_back(make_pair(lado,lado));*/
        }


    return waypints;

}

vector<pair<double, double>> Strategy::path_planningRRT(pair<double, double> root, pair<double, double> goal, Team obs, int idx_Robo, int numInt, double searchRaio)
{

    vector<vetor> rrt;  //arvore da rrt;
    vector<int> adjList; //lista de adjacências
    adjList.push_back(-1);
    rrt.push_back(root);

    double dist=distancia(root.first,root.second, goal.first,goal.second);
    if(dist > searchRaio)
        searchRaio = dist;

    goal = make_pair(root.first + searchRaio*cos(atan2(goal.second-root.second,goal.first-root.first)),
                     root.second + searchRaio*sin(atan2(goal.second-root.second,goal.first-root.first)));

    int currIdx=0; //indice atual
    int idxProxGoal=0; //indice mais proximo do objetivo

    double passo=0.10; //Passo de expansão da RRT

    double distMin=INFINITY;
    int i=0;
    for(i=0;i<numInt;i++)
    {
         vetor q_rand = gerarPontoAleatotio(root,goal,0.9,searchRaio);
         //cout<<q_rand.first<<", "<< q_rand.second<<endl;


         int id_qNear = buscaPontoMaisProximo(rrt,q_rand);

         vetor q_near = rrt.at(id_qNear);
         //cout<<q_near.first<<", "<< q_near.second<<endl;

         vetor q_new = gerarNovoPonto(q_near,q_rand,passo);
         //cout<<q_new.first<<", "<< q_new.second<<endl;

         if(regraDeExclusao(q_new,obs,idx_Robo))
             {
                 rrt.push_back(q_new);
                 adjList.push_back(id_qNear);
                 currIdx++;
                 //Escpaço para definir indixe do ponto mais proximo do objetivo
                 double distToGoal=distancia(rrt.at(currIdx).first,rrt.at(currIdx).second, goal.first,goal.second);
                 if(distToGoal<distMin)
                     {
                         distMin=distToGoal;
                         idxProxGoal=currIdx;
                     }

                 if(distToGoal<0.1)
                     break;

             }

    }
    cout << "Numero de Iterações: " << i <<endl;
    cout << "Tamanho do Caminho: " << rrt.size() <<endl;

    vector<vetor> path = gerarCaminho(rrt,adjList,idxProxGoal);
    cout << "Tamanho do Caminho2: " << path.size() <<endl;
    if(!path.size())
        path.push_back(root);

    return path;


}

pair<double, double> Strategy::gerarPontoAleatotio(pair<double, double> currPos, double raio)
{
    //double r = (rand()/RAND_MAX)*raio;
    //double theta = (rand()/RAND_MAX)*M_PI - (M_PI/2);

    double r= (static_cast <double> (rand())/(static_cast <double> (RAND_MAX/2*raio))) - raio;
    double theta= (static_cast <double> (rand())/(static_cast <double> (RAND_MAX/M_PI))) - M_PI/2;

    pair<double, double> q_rand;
    double x=currPos.first + r*cos(theta);
    double y=currPos.second + r*sin(theta);
    if((abs(x)>0.75))
        x=(0.75*x)/abs(x);

    if((abs(y)>0.65))
        y=(0.65*y)/abs(y);

    q_rand = make_pair(x,y);

    return q_rand;

}

pair<double, double> Strategy::gerarPontoAleatotio(pair<double, double> currPos, pair<double, double> goal, double prob, double raio)
{
    double r= (static_cast <double> (rand())/(static_cast <double> (RAND_MAX/2*raio))) - raio;
    //double theta= (static_cast <double> (rand())/(static_cast <double> (RAND_MAX/(M_PI)))) - M_PI/2;


    double ang = (180/M_PI)*atan2(goal.first-currPos.first,goal.second-currPos.second);
    double theta;
    if(ang < -90)
        {
            theta= -(static_cast <double> (rand())/(static_cast <double> (RAND_MAX/(M_PI/2)))) - M_PI/2;
        }
    else if(ang > -90 && ang < 0)
        {
           theta= -(static_cast <double> (rand())/(static_cast <double> (RAND_MAX/(M_PI/2))));
        }
    else if(ang>0 && ang<90)
        {
          theta= (static_cast <double> (rand())/(static_cast <double> (RAND_MAX/(M_PI/2))));
        }
    else if(ang>90 && ang<180)
        {
            theta= (static_cast <double> (rand())/(static_cast <double> (RAND_MAX/(M_PI/2)))) + M_PI/2;
        }
    else
        {
            theta= (static_cast <double> (rand())/(static_cast <double> (RAND_MAX/(M_PI)))) - M_PI/2;
        }

    pair<double, double> q_rand;
    double x=currPos.first + r*cos(theta);
    double y=currPos.second + r*sin(theta);

    //moeda viciada
    double p = static_cast <double> (rand());

    if(p>prob)
    {
        if((abs(x)>0.72))
            x=(0.72*x)/abs(x);

        if((abs(y)>0.62))
            y=(0.62*y)/abs(y);

        q_rand = make_pair(x,y);

    }else{
        q_rand = goal;

    }

    return q_rand;
}

int Strategy::buscaPontoMaisProximo(vector<pair<double, double>> arvore, pair<double, double> q_rand)
{
    int qtd_pontos_rrt = arvore.size();
    double min_dist=INFINITY;
    int idMaisProximo=0;
    for(int i=0;i<qtd_pontos_rrt;i++)
        {
            double dist = distancia(arvore.at(i).first,arvore.at(i).second,q_rand.first,q_rand.second);

            if(dist<min_dist)
                {
                    min_dist=dist;
                    idMaisProximo=i;
                }

        }
    return idMaisProximo;

}

pair<double, double> Strategy::gerarNovoPonto(pair<double, double> q_near, pair<double, double> q_rand, double passo)
{
    //Parâmetro de filtro
    double k=0.5;

    double norm = distancia(q_rand.first,q_rand.second,q_near.first,q_near.second);
    double qnew_x = q_near.first + passo*(q_rand.first - q_near.first)/norm;
    double qnew_y = q_near.second + passo*(q_rand.second - q_near.second)/norm;

    //filtragem
    qnew_x = q_near.first*(1-k) + qnew_x*k;
    qnew_y = q_near.second*(1-k) + qnew_y*k;

    if((abs(qnew_x)>0.75))
        qnew_x=(0.75*qnew_x)/abs(qnew_x);

    if((abs(qnew_y)>0.65))
        qnew_x=(0.65*qnew_y)/abs(qnew_y);


    return make_pair(qnew_x,qnew_y);

}

bool Strategy::regraDeExclusao(pair<double, double> q_new, Team obs, int idx)
{
    double raioRobo = 0.05;
    int numObs = obs.size();
    bool resultado=true;

    for(int i=0;i<numObs;i++)
        {
            if(i!=idx)
            {
                double dist=distancia(q_new.first,q_new.second,obs.at(i).x(),obs.at(i).y());
                if(dist<(2.2*raioRobo))
                    {
                        resultado=false;
                        break;
                    }
            }
        }
    return resultado;
}

vector<pair<double, double> > Strategy::gerarCaminho(vector<pair<double, double>> arvore, vector<int> adjList, int idxGoal)
{
    int pivo=idxGoal;
    vector<pair<double, double>> path;
    path.clear();
    while(pivo>0)
    {
        path.push_back(make_pair(arvore.at(pivo).first,arvore.at(pivo).second));
        pivo = adjList.at(pivo);
    }
    vector<pair<double, double>> rpath;
    rpath.clear();
    for(int i=path.size()-1;i>=0;i--)
        {
            rpath.push_back(path.at(i));
        }

    if(rpath.size()>1)
        rpath.pop_back();

    return rpath;
}

bool Strategy::straitlineTest(pair<double, double> currPos,pair<double, double> target, Team obs, int idx)
{
    //vetor 'V'
         pair<double, double> V = make_pair(target.first-currPos.first,target.second-currPos.second);
         //Norma do vetor 'V'
         double norm_V = sqrt(pow(V.first,2.0)+pow(V.second,2.0));
         double dist=INFINITY;

         for(int i = 0; i<(int)obs.size();i++)
         {
             //vetor 'AP'
             if(i!=idx)
             {
                pair<double, double> AP = make_pair(obs[i].x()-currPos.first,obs[i].y()-currPos.first);
                 //Norma do produto vetorial VxAP
                dist = abs((V.first*AP.second)-(V.second*AP.first))/norm_V;
             }

             if(dist < 0.15)
                 return false;

         }
         return true;

}

bool Strategy::dotCriterio(pair<double, double> currPos, pair<double, double> carrotP, pair<double, double> goalP)
{

    double vx = currPos.first-carrotP.first;
    double vy = currPos.second-carrotP.second;
    double ux = goalP.first-carrotP.first;
    double uy = goalP.second-carrotP.second;

    //Se cri>0 -> replaneja
    double cri = ux*vx + uy*vy;

    if(cri>0)
        return true;
    else
        return false;

}

void Strategy::vaiParaRRT(fira_message::Robot robot, int id_robot, pair<double, double> goalP,pair<double, double> bola)
{
    double Curr_velR1 = sqrt(pow(robot.vx(), 2)+pow(robot.vy(), 2));

    double l_ahead=0.07;   //Look-Ahead distance

    double raio_rrt=0.45;  //Raio da RRT

    double a = 0.5; //frequência de corte
    pair<double,double> currPos=make_pair(robot.x(),robot.y());

    if(replain)
        {
             start=clock();
            //caminho=takeBallToGoal(make_pair(b1.x(),b1.y()),make_pair(ball.x(),ball.y()));
            //caminho=gerar_caminho(100);
              caminho.clear();
              while (!(caminho.size()>1))
                 {
                    if(plain && (Curr_velR1>0.015))
                       currPos=carrot_point;

                    caminho = path_planningRRT(currPos,bola,robots,1,300,raio_rrt);
                    filtro_caminho(a);
                    setup_pure_pursuit(caminho.at(0));

                 }

              plain = true;
              replain=false;

              end = clock();

              tempo=pow(10,3)*(end-start)/CLOCKS_PER_SEC;
              cout << "Tempo de Planejamento: " << tempo << "ms "<<endl;

        }

    //critério do produto escalar (repleneja se a bola estiver muito desvida do objetivo)
    bool cri = dotCriterio(currPos,carrot_point,goalP);

    if((Curr_velR1<0.015)||(cri==true))
        replain=true;

    double v_pref = 0.35;

    //pure_pursuit(robot,id_robot,caminho,l_ahead,v_pref);
    ang_err err_goal = turnToDestination(robot,bola,goalP);
    pure_pursuit2(robot,id_robot,caminho,err_goal);


    //Envio de dados para o plot
    send_data_control(v_pref,Curr_velR1);

}

void Strategy::takeBallToGoal(fira_message::Robot robot,int id_robot, pair<double,double> ballPos)
{

    //Comportamento simples com muito a ser aprimorado

    double Curr_velR1 = sqrt(pow(robot.vx(), 2)+pow(robot.vy(), 2));

    double l_ahead=0.07;   //Look-Ahead distance

    double raio_rrt=2*l_ahead;  //Raio da RRT

    double a = 0.7; //frequência de corte
    pair<double,double> currPos=make_pair(robot.x(),robot.y());

    double raio=3*l_ahead;
    double th = atan2(-ballPos.second,-0.72*lado - ballPos.first) - (abs(ballPos.second)/0.65)*(ballPos.second/abs(ballPos.second))*M_PI/2;
    pair<double, double> goalP = make_pair(ballPos.first-raio*cos(th),ballPos.second-raio*sin(th));

    double v_pref = 0.5;

    if(distancia(currPos.first,currPos.second,goalP.first,goalP.second)<l_ahead+0.15)
    {
        if(flagTbT)
            {
                caminho.clear();
                caminho.push_back(goalP);
                caminho.push_back(ballPos);
                caminho.push_back(make_pair(-0.72*lado,0));
                setup_pure_pursuit(caminho.at(0));
                flagTbT=false;
            }

    }else if(replain)
        {
             start=clock();
              caminho.clear();
              while (!(caminho.size()>1))
                 {
                    pair<double,double> root = currPos;
                    if(plain && (Curr_velR1>0.01))
                        root=carrot_point;

                    caminho = path_planningRRT(root,goalP,robots,1,200,raio_rrt);
                    filtro_caminho(a);
                    if(caminho.size()>0)
                        setup_pure_pursuit(caminho.at(0));

                 }

              plain = true;
              replain=false;
              flagTbT=true;

              end = clock();

              tempo=pow(10,3)*(end-start)/CLOCKS_PER_SEC;
              cout << "Tempo de Planejamento: " << tempo << "ms "<<endl;

        }

    //critério do produto escalar (repleneja se a bola estiver muito desvida do objetivo)
    bool cri = dotCriterio(currPos,carrot_point,goalP);
    //cri = false;
    if(Curr_velR1<0.01||cri==true)
        replain=true;


    pure_pursuit(robot,id_robot,caminho,l_ahead,v_pref);

    //pure_pursuit2(robot,id_robot,caminho,goalP);

}
ang_err Strategy::turnToDestination(fira_message::Robot robot, pair<double, double> ball, pair<double, double> dest)
{

    //Comportamento simples com muito a ser aprimorado

    double dist = sqrt(pow(ball.first-robot.x(),2)+pow(ball.second-robot.y(),2));
    ang_err err_goal;
    cout << "distancia para a bola: " << dist <<endl;


    if(dist<=0.1)//0.003
        //double err_goal = atan2(ball.second-dest.second,ball.first-dest.first)*(180/M_PI);
        err_goal = olhar(robot,dest.first,dest.second);
    else
    {
        //ang_err err_goal = olhar(robot,robot.x(),robot.y()); // Zero
        err_goal.fi = 0.0;
        err_goal.flag = 0;
    }


    //pair<double, double> goal_p = make_pair(dest.first,dest.second);

    cout << "Erro par o gol: " << err_goal.fi <<endl;
    return err_goal;

    // talvez tenha influnêcia na movimentação lateral da bola em relação ao robô
    // verificar efeito no replanejamento, pois pode requerer a associação de um novo comportamento independente da rrt

}

void Strategy::PENALTY_KICK(int id, ang_err angulo)
{
    VW[id][0] = angulo.flag*5;
    VW[id][1] = 0;
}

void Strategy::pure_pursuit(fira_message::Robot robot, int id_robot,vector<pair<double,double>> points, double lookAhead_dist, double v_pref)
{
    //Calcular distâncias do robô aos pontos no caminho

    //evitar estouro de vetor
    int treshold = points.size();
    if(pivot_pp > treshold-1)
    {
        pivot_pp=treshold-1;
        cout << "Pivo: "<< pivot_pp<<endl;
    }


    ang_err angulo = olhar(robot, carrot_point.first, carrot_point.second);
   // v_pref=v_pref*abs(cos(angulo.fi*M_PI/180));
   if((abs(angulo.fi) > 45)&&(abs(angulo.fi) < 135))
         v_pref=0;

    double dt = 0.016;
    pair<double,double> v_swp = sweep_path(carrot_point, points.at(pivot_pp),v_pref);
    carrot_point.first = carrot_point.first + v_swp.first*dt;
    carrot_point.second = carrot_point.second + v_swp.second*dt;


    double dd = sqrt(pow(points.at(pivot_pp).first-carrot_point.first,2)+
                     pow(points.at(pivot_pp).second-carrot_point.second,2));

    double d_acc = 0.05;

    if( (dd < d_acc)&&(pivot_pp<treshold-1))
        pivot_pp=pivot_pp+1;
    else if(pivot_pp>=treshold-1)
      {
        replain=true;  // habilita replanejamento
      }

   /* double err = sqrt(pow(robot.x()-carrot_point.first,2)+
                      pow(robot.y()-carrot_point.second,2))-lookAhead_dist;*/

    //VW[id_robot][0] = controleLinear(err,3.65,0.01);
    VW[id_robot][0] = controleLinear(robot, carrot_point.first,carrot_point.second,lookAhead_dist);
    angulo = olhar(robot, carrot_point.first, carrot_point.second);
    VW[id_robot][1] = controleAngular(angulo.fi);

    //send_data_control(sqrt(pow(v_swp.first, 2)+pow(v_swp.second, 2)),sqrt(pow(robot.vx(), 2)+pow(robot.vy(), 2)));


}

void Strategy::pure_pursuit2(fira_message::Robot robot, int id_robot, vector<pair<double, double> > points, ang_err err_to_goal)
{
    //Calcular distâncias do robô aos pontos no caminho

    //evitar estouro de vetor
    int treshold = points.size();
    if(pivot_pp > treshold-1)
    {
        pivot_pp=treshold-1;
        cout << "Pivo: "<< pivot_pp<<endl;
    }



    double dd = sqrt(pow(points.at(pivot_pp).first-robot.x(),2)+
                     pow(points.at(pivot_pp).second-robot.y(),2));

    double d_acc = 0.05;

    if( (dd < d_acc)&&(pivot_pp<treshold-1))
        pivot_pp=pivot_pp+1;
    else if(pivot_pp>=treshold-1)
      {
        replain=true;  // habilita replanejamento
      }

    ang_err angulo = olhar(robot, points.at(pivot_pp).first, points.at(pivot_pp).second);
    VW[id_robot][0] = 0.25*Vmax*angulo.flag;
    VW[id_robot][1] = controleAngular(angulo.fi + err_to_goal.fi);
}

pair<double, double> Strategy::sweep_path(pair<double, double> goal_p, pair<double, double> ref_p, double v_pref)
{
    double norm_factor = sqrt(pow(ref_p.first-goal_p.first,2) + pow(ref_p.second-goal_p.second,2));
    double x_p = v_pref*((ref_p.first-goal_p.first)/norm_factor);
    double y_p = v_pref*((ref_p.second-goal_p.second)/norm_factor);
    return make_pair(x_p,y_p);
}

double Strategy::controleLinear(double err, double Kp, double Ki)
{
     err_i = err_i + err*0.016;
     return  Kp*err + Ki*err_i;
}

void Strategy::setup_pure_pursuit(pair<double,double> p)
{
    this->err_i=0;
    this->carrot_point=p;
    this->pivot_pp=1;
}

void Strategy::send_data_robots(Team rb, QString name)
{
    QFile file(name);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
     QTextStream out(&file);
     //int qtd=rb.size();
       for(int i=0; i<6;i++)
           out<<" "<<rb[i].x()<<" "<<rb[i].y()<< " "<<rb[i].orientation()<<"";
    }
    file.close();
}

void Strategy::send_data_ball(double x, double y, QString name)
{
    QFile file(name);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
     QTextStream out(&file);

       out<<x<<" "<<y<< "";
    }
    file.close();
}

void Strategy::send_data_path(vector<pair<double, double>> pontos, QString name)
{

    QFile file(name);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
     QTextStream out(&file);
     int k=pontos.size();
     for(int i=0; i<k;i++)
         out<<" "<<pontos[i].first<<" "<<pontos[i].second<<"";
    }
    file.close();
}

void Strategy::send_data_control(double setPoint, double varControl, QString name)
{
    QFile file(name);
    if(file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        QTextStream out(&file);
        out<<setPoint<<" "<<varControl<<"";
    }
    file.close();
}

Strategy::~Strategy()
{

}

//Alterações Petersson
//Função de saturação dos valores a serem enviados aos vaipara
void Strategy::saturacao(double V[]){
    //limites de x e y
    double lim_x = 0.68;
    double lim_y = 0.58;

    //Satura as posições enviadas
    if (V[0] > lim_x)
        V[0] = lim_x;

    if (V[0] < -lim_x)
        V[0] = -lim_x;

    if (V[1] > lim_y)
        V[1] = lim_y;

    if (V[1] < -lim_y)
        V[1] = -lim_y;
}

void Strategy::saturacao(vector <double> *V){
    //limites de x e y
    double lim_x = 0.68;
    double lim_y = 0.58;

    //Satura as posições enviadas
    if (V->at(0) > lim_x)
        V->at(0) = lim_x;

    if (V->at(0) < -lim_x)
        V->at(0) = -lim_x;

    if (V->at(1) > lim_y)
        V->at(1) = lim_y;

    if (V->at(1) < -lim_y)
        V->at(1) = -lim_y;
}

//Atualiza a estrutura de dados contendo as posições de todos os robôs
void Strategy::atualiza_pos(fira_message::Robot b0,fira_message::Robot b1,fira_message::Robot b2,fira_message::Robot y0,fira_message::Robot y1,fira_message::Robot y2){
    Strategy::pos_robos[0][0] = b0.x();
    Strategy::pos_robos[0][1] = b0.y();
    Strategy::pos_robos[1][0] = b1.x();
    Strategy::pos_robos[1][1] = b1.y();
    Strategy::pos_robos[2][0] = b2.x();
    Strategy::pos_robos[2][1] = b2.y();

    Strategy::pos_robos[3][0] = y0.x();
    Strategy::pos_robos[3][1] = y0.y();
    Strategy::pos_robos[4][0] = y1.x();
    Strategy::pos_robos[4][1] = y1.y();
    Strategy::pos_robos[5][0] = y2.x();
    Strategy::pos_robos[5][1] = y2.y();

}


//Campo repulsivo linear
void Strategy::calc_repulsao(fira_message::Robot rb, double F[]){
     double raio_adversario = 0.2;  //raio de detecção do adversario

     F[0] = 0;
     F[1] = 0;

     double dist_adversario;

     for(int i = 0 ; i < 5 ; i++){

         dist_adversario = sqrt(pow((rb.x() - pos_robos[i][0]),2) + pow((rb.y() - pos_robos[i][1]),2));

         if (dist_adversario <= raio_adversario && dist_adversario > 0.01){
             double dist_x = rb.x() - pos_robos[i][0];
             double dist_y = rb.y() - pos_robos[i][1];

             F[0] += raio_adversario*dist_x/dist_adversario - dist_x;
             F[1] += raio_adversario*dist_y/dist_adversario - dist_y;
         }

     }
}

//limita o valor de um vetor
void Strategy::converte_vetor(double V[],double raio){

    double dist = sqrt(pow(V[0],2) + pow(V[1],2));

    if (dist > raio){
        V[0] = raio*V[0]/dist;
        V[1] = raio*V[1]/dist;
    }

}

double Strategy::filtro(double V,int id){

    //filtro IIR
    double k = 0.9;

    V = velocidades[id]*(1-k) + V*k;
    velocidades[id] = V;

    return V;
}

void Strategy::goleiro_petersson(fira_message::Robot rb,fira_message::Ball ball, int id){

    double top_limit = 0.17; //largura do gol/2
    double x_desejado = -0.7*lado;

    double dist = distancia(rb,ball.x(),ball.y());
    vector <double> new_pos = {0,0};

    //se a bola estiver longe utiliza o preditor
    if ( dist > 0.3){
        new_pos = {predictedBall.x,predictedBall.y};
    }else{
        new_pos = {ball.x(),ball.y()};
    }

    if(distancia(rb,x_desejado,rb.y()) >= 0.02){ //se o robô está dentro do retângulo

        if(distancia(rb,x_desejado,rb.y()) >= 0.3){
              vaiPara_desviando(rb,x_desejado,0.0,id);
        }else{
              vaiPara(rb,x_desejado,0.0,id);
        }

    }else{

        ang_err angulo = olhar(rb,rb.x(),top_limit + 5); // calcula diferença entre angulo atual e angulo desejado
        if(angulo.fi >= 0.5 || angulo.fi<= -0.5){ //se o robô não está aproximadamente 90 graus
            andarFrente(0,id);
            VW[id][1] = controleAngular(angulo.fi);
        }

        else if(rb.y() < top_limit && rb.y() < new_pos[1]){ //robô abaixo da bola

            if(angulo.flag == 1){
                andarFrente(125,id);
            }
            else{
                andarFundo(125,id);
            }
        }
        else if(rb.y() > -top_limit && rb.y() > new_pos[1]){ //robô acima da bola
            if(angulo.flag == 1){
                andarFundo(125,id);
            }
            else{
                andarFrente(125,id);
            }
        }
        else{
            andarFrente(0,id);
        }
        //gira se a bola estiver muito perto
        if (distancia(rb,ball.x(),ball.y()) < 0.08){
            if((ball.y() < 0 && lado == 1)){
               girarHorario(125,id);
            }
            if((ball.y() > 0 && lado == -1)){
               girarHorario(125,id);
            }
            if((ball.y() > 0 && lado == 1)){
               girarAntihorario(125,id);
            }
            if((ball.y() < 0 && lado == -1)){
               girarAntihorario(125,id);
            }
        }
    }

}

void Strategy::vaiPara_desviando(fira_message::Robot rb,double px,double py,int id){

    double V[2] = {px - rb.x(),py - rb.y()};

    double F[2] = {0,0};

    calc_repulsao(rb,F);

    double ka = 1;
    double kr = 1;

    converte_vetor(V,0.1);
    converte_vetor(F,0.2);

    double new_pos[2] = {rb.x() + ka*V[0] + kr*F[0],rb.y() + ka*V[1] + kr*F[1]};

    Strategy::saturacao(new_pos);

    vaiPara(rb,new_pos[0],new_pos[1],id);

    filtro(VW[id][0],id);
}


void Strategy::zagueiro2(fira_message::Robot rb, fira_message::Ball ball, int id){
    double xbola = ball.x();
    double ybola = ball.y();
    double x_penalti =  0.4*lado; //bolinha do penalti
       double x_meio_de_campo = 0.0; //centro do campo
       double x_radius = 0.2*lado; //raio do circulo do meio de campo
       double y_top = 0.65; // metade da largura do campo
       double ala_deepth = 0.35; //metade da área do goleiro
       double K_press = 0.3*lado; //ṕonto entre meio de campo e marca do penalti
       double ajuste = 0.15; //ajuste de lugar onde o zagueiro, quando a bola está proximo do escanteio do seu time
       double ajuste2 = 0.1; //ajuste de distância que o zagueiro deve estar para ir na bola no campo de ataque
       int teste = 0; //armazena se entrou no else
       double deltaY = 0.1; //distancia em Y que faz ele acompanhar ao invés de olhar para a bola

       if(lado == 1){
           //lado azul
           if(xbola > x_penalti)
           {    //Se a Bola estiver na zona "A"
               if (abs(rb.y()-ball.y()) < deltaY) //a bola está num y próximo
               {
                   VW[id][0] = 0;
                   VW[id][1] = controleAngular(olhar(rb,ball.x(),ball.y()).fi);
               }
               else
               {
                   vaiPara_desviando(rb,x_meio_de_campo + K_press,ybola,id);
                   if((distancia(rb,xbola,ybola)< ajuste2) && rb.x() < xbola){
                       //Se o robo estiver perto da bola e atrás dela
                       vaiPara(rb,predictedBall.x,predictedBall.y,id);
                   }
               }
           }
           else if(xbola >= x_meio_de_campo && (ybola < (y_top - ala_deepth) && ybola > (ala_deepth - y_top)))
           {    //Se a Bola estiver na zona "B_mid"
               vaiPara_desviando(rb,-x_penalti,ybola,id);
               if((distancia(rb,xbola,ybola)< ajuste2) && rb.x() < xbola){
                   //Se o robo estiver perto da bola e atrás dela
                   vaiPara(rb,predictedBall.x,predictedBall.y,id);
               }
           }
           else if((xbola >= x_meio_de_campo) && (ybola > (y_top - ala_deepth) || ybola < (ala_deepth - y_top)))
           {    //Se a Bola estiver na zona "B_top" ou "B_bot"
               vaiPara_desviando(rb,-x_radius,ybola,id);
           }
           else if(xbola < x_meio_de_campo && xbola > rb.x() && (ybola < (y_top - ala_deepth) && ybola > (ala_deepth - y_top)))
           {    //Se a Bola estiver na zona "C"
               vaiPara_desviando(rb,xbola,ybola,id);
           }
           else if((xbola < x_meio_de_campo && ybola > (y_top - ala_deepth +ajuste)))
           {   //Se a Bola estiver na zona "D"
               if((predictedBall.x - xbola) < 0){//se a bola estiver indo para o lado esquerdo
                   vaiPara_desviando(rb,predictedBall.x,y_top - ala_deepth + ajuste,id);
               }else{
               vaiPara_desviando(rb,xbola,y_top - ala_deepth + ajuste,id);
               }
                if(rb.x() - 0.05 < xbola){//Se o robô está atras da bola
                    vaiPara_desviando(rb,xbola,ybola,id);
                    //vaiParaRRT(rb,id,make_pair(xbola,ybola));
                }
           }
           else if((xbola < x_meio_de_campo) && ybola < (ala_deepth - y_top  -ajuste))
           {    //Se a Bola estiver na zona "E"
               if((predictedBall.x - xbola) < 0){//se a bola estiver indo para o lado direito
                   vaiPara_desviando(rb,predictedBall.x,ala_deepth - y_top - ajuste,id);
               }else{
                   vaiPara_desviando(rb,xbola,ala_deepth - y_top - ajuste,id);
               }
               if(rb.x() -0.05 < xbola){//Se o robô está atras da bola
                   vaiPara_desviando(rb,xbola,ybola,id);
               }
           }
           else // se não estiver em nenhuma das regiôes
           {
               if (ybola>0.15)
               {
                   vaiPara(rb,-x_penalti - 0.1, 0.15,id);
               }
               else if (ybola<-0.15)
               {
                   vaiPara(rb,-x_penalti - 0.1, -0.15,id);
               }
               else
               {
                   vaiPara_desviando2(rb,-x_penalti - 0.1, 0,id);
               }
               teste = 1; //entrou aqui
               /*if(sqrt( pow((-x_penalti -0.1 -predictedBall.x),2) + pow((0.0 - predictedBall.y),2) ) < 0.2){
                   vaiPara_desviando(rb,-x_penalti +0.1, 0.0,id);
               }*/
           }
           //gira se a bola estiver muito perto
           if ((distancia(rb,xbola,ybola) < 0.08) && (((rb.x() < xbola)&&teste == 0))){
               if(ybola < 0){
                  girarHorario(125,id);
               }
               if(ybola > 0){
                  girarAntihorario(125,id);
               }
           }
        }else{
           //lado amarelo
           if(xbola < x_penalti)
           {    //Se a Bola estiver na zona "A"
               if (abs(rb.y()-ball.y()) < deltaY) //a bola está num y próximo
               {
                   VW[id][0] = 0;
                   VW[id][1] = controleAngular(olhar(rb,ball.x(),ball.y()).fi);
               }
               else
               {
                   vaiPara_desviando(rb,x_meio_de_campo + K_press,ybola,id);
                   if((distancia(rb,xbola,ybola)< ajuste2) && rb.x() > xbola){
                       //Se o robo estiver perto da bola e atrás dela
                       vaiPara(rb,predictedBall.x,predictedBall.y,id);
                    }
               }
           }
           else if(xbola <= x_meio_de_campo && (ybola < (y_top - ala_deepth) && ybola > (ala_deepth - y_top)))
           {    //Se a Bola estiver na zona "B_mid"
               vaiPara_desviando(rb,-x_penalti,ybola,id);
               if((distancia(rb,xbola,ybola)< ajuste2) && rb.x() > xbola){
                   //Se o robo estiver perto da bola e atrás dela
                   vaiPara(rb,predictedBall.x,predictedBall.y,id);
               }
           }
           else if((xbola <= x_meio_de_campo) && (ybola > (y_top - ala_deepth) || ybola < (ala_deepth - y_top)))
           {    //Se a Bola estiver na zona "B_top" ou "B_bot"
               vaiPara_desviando(rb,-x_radius,ybola,id);
           }
           else if(xbola > x_meio_de_campo && xbola < rb.x() && (ybola < (y_top - ala_deepth) && ybola > (ala_deepth - y_top)))
           {    //Se a Bola estiver na zona "C"
               vaiPara_desviando(rb,xbola,ybola,id);
           }
           else if((xbola > x_meio_de_campo && ybola > (y_top - ala_deepth +ajuste)))
           {   //Se a Bola estiver na zona "D"
               if((predictedBall.x - xbola) > 0){//se a bola estiver indo para o lado direito
                   vaiPara_desviando(rb,predictedBall.x,y_top - ala_deepth + ajuste,id);
               }else{
               vaiPara_desviando(rb,xbola,y_top - ala_deepth + ajuste,id);
               }
                if(rb.x() + 0.05 > xbola){//Se o robô está atras da bola
                    vaiPara_desviando(rb,xbola,ybola,id);
                }
           }
           else if((xbola > x_meio_de_campo) && ybola < (ala_deepth - y_top  -ajuste))
           {    //Se a Bola estiver na zona "E"
               if((predictedBall.x - xbola) > 0){//se a bola estiver indo para o lado direito
                   vaiPara_desviando(rb,predictedBall.x,ala_deepth - y_top - ajuste,id);
               }else{
                   vaiPara_desviando(rb,xbola,ala_deepth - y_top - ajuste,id);
               }
               if(rb.x() +0.05 > xbola){//Se o robô está atras da bola
                   vaiPara_desviando(rb,xbola,ybola,id);
               }
           }
           else // se não estiver em nenhuma das regiôes
           {
               if (ybola>0.15)
               {
                   vaiPara(rb,-x_penalti + 0.1, 0.15,id);
               }
               else if (ybola<-0.15)
               {
                   vaiPara(rb,-x_penalti + 0.1, -0.15,id);
               }
               else
               {
                   vaiPara_desviando2(rb,-x_penalti + 0.1, 0,id);
               }

               teste = 1;
               /*if(sqrt( pow((-x_penalti +0.1 -predictedBall.x),2) + pow((0.0 - predictedBall.y),2) ) < 0.2){
                   vaiPara_desviando(rb,-x_penalti -0.1, 0.0,id);
               }*/
           }
           //gira se a bola estiver muito perto
           if ((distancia(rb,xbola,ybola) < 0.08) && ((rb.x() > xbola)&&teste == 0)){
               if(ybola > 0){
                  girarHorario(125,id);
               }
               if(ybola < 0){
                  girarAntihorario(125,id);
               }
           }
       }
       //se tiver uma reta clara pro gol, ele vai chutar
       FIRE_KICK(rb,ball,id);
}




//serve apenas para o zagueiro2
void Strategy::vaiPara_desviando2(fira_message::Robot rb,double px,double py,int id){

    double V[2] = {px - rb.x(),py - rb.y()};

    double F[2] = {0,0};

    calc_repulsao2(rb,F);

    double ka = 1;
    double kr = 1;

    converte_vetor(V,0.1);
    converte_vetor(F,0.2);

    double new_pos[2] = {rb.x() + ka*V[0] + kr*F[0],rb.y() + ka*V[1] + kr*F[1]};

    Strategy::saturacao(new_pos);

    vaiPara(rb,new_pos[0],new_pos[1],id);

    filtro(VW[id][0],id);
}

//Campo repulsivo linear
//serve apenas para o zagueiro2
void Strategy::calc_repulsao2(fira_message::Robot rb, double F[]){
    double raio_adversario = 0.2;  //raio de detecção do adversario

         F[0] = 0;
         F[1] = 0;

         double dist_adversario;

         for(int i = 0 ; i < 5 ; i++){

             dist_adversario = sqrt(pow((rb.x() - pos_robos[i][0]),2) + pow((rb.y() - pos_robos[i][1]),2));

             if (dist_adversario <= raio_adversario && dist_adversario > 0.01){
                 double dist_x = rb.x() - pos_robos[i][0];
                 double dist_y = rb.y() - pos_robos[i][1];

                 F[0] += raio_adversario*dist_x/dist_adversario - dist_x;
                 F[1] += raio_adversario*dist_y/dist_adversario - dist_y;
             }

         }
         dist_adversario = sqrt(pow((rb.x() - predictedBall.x),2) + pow((rb.y() - predictedBall.y),2));
         if (dist_adversario <= raio_adversario){
             raio_adversario = 0.25;
             double dist_x = rb.x() - predictedBall.x;
             double dist_y = rb.y() - predictedBall.y;

             F[0] += raio_adversario*dist_x/dist_adversario - dist_x;
             F[1] += raio_adversario*dist_y/dist_adversario - dist_y;
         }
}

//Atacante de Lázaro e cone
void Strategy::atacante_todos(Team rb,Team adversario, fira_message::Ball ball, int id, int idzag)
{

    double alpha = M_PI/8;
    double meioGolx = -0.75*lado;
    double golSup = 0.20;
    double goalInf = -0.20;
    double beta  = atan2(rb[id].x() - meioGolx , rb[id].y());
    double gamma = atan2(rb[id].x() - ball.x() , rb[id].y() - ball.y());
    double roh   = atan2(golSup - rb[id].x(), golSup - rb[id].y());
    double omega = atan2(goalInf - rb[id].x(), goalInf - rb[id].y());
    int K = round(beta/alpha);
    double gain;
  //double resultante_2 = {0,0};

    if(K<2)
        gain = 0.4;
    else if(K>=2 && K <= 3)
        gain = 0.35;
    else if(K>3 && K < 5)
        gain = 0.4;
    else if(K>=5 && K < 7)
        gain = 0.35;
    else
        gain = 0.4;

    delete resultante_2;
    resultante_2 = new vector<double>();

    //Determinação da origem do vetor resultante
    resultante_2->push_back(rb[id].x());
    resultante_2->push_back(rb[id].y());

    //distância atacante-bola
    double dist =  sqrt(pow(ball.x()-rb[id].x(),2.0)+pow( ball.y()-rb[id].y(),2.0));
    //Distancia do zagueiro até a bola
    double zag_dist = sqrt(pow(ball.x()-rb[idzag].x(),2.0)+pow( ball.y()-rb[idzag].y(),2.0));

    double componenteX;
    double componenteY;

    double k = dist;
    if(dist < 0.2)
        k  = 0.2;

    (*resultante_2)[0]+=-k*sin(gamma);
    (*resultante_2)[1]+=-k*cos(gamma);


    if(lado > 0){//azul


        if(ball.x() < rb[id].x())
        {
            int a = 0;
            //Consideração do vetor de predição
            if(abs(ball.y()-rb[id].y()) > 0.3)
                a = 1;
            else if(abs(ball.y()-rb[id].y()) < 0.06 && dist < 0.2)
            {
                (*resultante_2)[0]=rb[id].x();
                (*resultante_2)[1]=rb[id].y();
            }

            int c = 0;
            if((ball.x()/rb[id].x()<1 && ball.x()/rb[id].x()>0.8) && abs(ball.y()-rb[id].y()) > 0.1)
                c = 1;


            double repulsivoX;
            double repulsivoY;

            if(ball.y() > rb[id].y())
            {
                repulsivoX = (c*0.1 + 0.1)*sin(0 + (c*0.5*M_PI));
                repulsivoY = (c*0.1 + 0.1)*cos(0 + (c*0.5*M_PI));
            }else
            {
                repulsivoX = (c*0.1+0.1)*sin(M_PI-(c*0.5*M_PI));
                repulsivoY = (c*0.1+0.1)*cos(M_PI-(c*0.5*M_PI));
            }
            //Componentes do Vetor direção
            double angle = atan2(ball.x()-predictedBall.x,ball.y()-predictedBall.y);
            componenteX = /*-c*ball.x()*/ -  a*0.1*sin(angle) - repulsivoX;
            componenteY = /*-c*ball.y()*/ -  a*0.1*cos(angle) - repulsivoY;

            (*resultante_2)[0]+= componenteX;
            (*resultante_2)[1]+= componenteY;

        }else
        {

            if(dist < 0.08)
            {
                (*resultante_2)[0]+=-gain*sin(beta+M_PI);
                (*resultante_2)[1]+=-gain*cos(beta+M_PI);
            }

            else{
                if((ball.x() < 0.6 && ball.y() < -0.2)&& (rb[id].y() > ball.y())/*&&(abs(ball.y()-blue[id].x())>0.07)*/)
                {
                    //Vetor de corrção
                    componenteX = /*ball.x()*/ -  0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);
                    componenteY = /*ball.y()*/ -  0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);

                    (*resultante_2)[0]+=componenteX;
                    (*resultante_2)[1]+=componenteY;

                }
                else if((ball.x() < 0.6 && ball.y() > 0.2)&&(rb[id].y() < ball.y())/*&&(abs(ball.y()-blue[id].x())>0.07)*/)
                {
                    //Vetor de corrção
                    componenteX = /*ball.x()*/ -  0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);
                    componenteY = /*ball.y()*/ -  0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);

                    (*resultante_2)[0]+=componenteX;
                    (*resultante_2)[1]+=componenteY;

                }
                else if(dist > 0.2)
                {
                    (*resultante_2)[0]+= -0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0));
                    (*resultante_2)[1]+= -0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0));
                }
                //atualização
                if(((ball.y()>0.25 && ball.y()<0.6) && ball.x() > 0.6 && rb[id].x() >= ball.x() && rb[id].y()>0.25 && rb[id].y()<ball.y())
                        ||( (ball.y()<-0.25 && ball.y()>-0.6) && ball.x() > 0.6 && rb[id].x() >= ball.x() && rb[id].y()<-0.25 && rb[id].y()>ball.y()))
                {
                    (*resultante_2)[0]+= -0.1*sin(M_PI/2);
                    (*resultante_2)[1]+= -0.1*cos(M_PI/2);
                }//
            }

        }

        saturacao(resultante_2);

        if(ball.x()==0)
        {
            vaiPara(rb[id],ball.x(),ball.y(),id);

        }
        else
        {
            if(ball.x()<=0)
            {

                if(zag_dist/dist < 0.9)
                {
                    //Comportamento sem bola : Defensivo

                    /* dist_target = distância do alvo para a bola;
                     * target = alvo da marcação;
                     * (zag_dist/dist) = razão entre as distâcias do zagueiro para
                     o atacante;
                     */

                    //Calculo do alvo de marcação
                    double dist_target = 0.0;
                    int target = 0;
                    for(int player = 0; player < 3; player++)
                    {
                        double dist_adv = sqrt(pow(ball.x()-adversario[player].x(),2.0)+pow( ball.y()-adversario[player].y(),2.0));
                        if(!(adversario[player].x() > 0.65 || adversario[player].x() < -0.65) && (dist_adv > dist_target))
                        {
                            dist_target = dist_adv;
                            target = player;
                        }
                    }

                    //Pos com relação ball-alvo
                    double attachX = ball.x() - (0.8*dist_target)*sin(atan2(ball.x() - adversario[target].x(),ball.y() - adversario[target].y()));
                    double attachY = ball.y() - (0.8*dist_target)*cos(atan2(ball.x() - adversario[target].x(),ball.y() - adversario[target].y()));

                    vaiPara_desviando(rb[id],attachX,attachY,id);
                    //vaiParaRRT(rb[id],id,make_pair(attachX,attachY),make_pair(-0.76,0));
                }
                else
                {
                    vaiPara_desviando(rb[id],0.10,(*resultante_2)[1]-0.1,id);
                }


            }else
            {
                //Comportamento Ofensivo - Posicionamento

               //Cálculo da posição
                double attachX =  ball.x() + 0.45*sin(atan2(ball.x() - rb[idzag].x(),ball.y() - rb[idzag].y()));
                //double attachY =  ball.y() + 0.45*cos(atan2(ball.x() - rb[1].x(),ball.y() - rb[1].y()));
                int c;
                if(rb[idzag].y()>0)
                    c = -1;
                else
                    c = 1;

                if(zag_dist/dist < 0.3)
                    vaiPara_desviando(rb[id],attachX-0.1,c*0.25,id);
                else
                {
                    vaiPara_desviando(rb[id],(*resultante_2)[0],(*resultante_2)[1],id);
                    //vaiParaRRT(rb[id],id,make_pair((*resultante_2)[0],(*resultante_2)[1]),make_pair(-0.76,0));

                }

                /*double distToGoal = sqrt(pow(rb[id].x()-meioGolx,2.0)+pow(rb[id].y()-0,2.0));
                if(distToGoal < 0.5 && ball.x()>rb[id].x() && (ball.y() < 0.20 && ball.y()>-0.20) )
                    vaiPara(rb[id],ball.x(),ball.y(),id);*/


                /*if(abs(rb[id].orientation()-gamma)<0.4 && (omega < gamma && gamma < roh))
                {
                    FIRE_KICK(id);
                    bandeira = false;
                }else
                {
                    bandeira = true;
                }*/

                if(ball.x() > 0.65 && rb[id].x() > 0.65 && dist < 0.08 && (ball.y() < 0.18 && ball.y() > -0.18)&& rb[id].y()>ball.y())
                {
                    //vaiPara(rb[id],(*resultante_2)[0],(*resultante_2)[1],id);
                    chute(id,1);

                }else if(ball.x() > 0.65 && rb[id].x() > 0.65 && dist < 0.08 && (ball.y() < 0.18 && ball.y() > -0.18)&& rb[id].y()<ball.y())
                {
                    chute(id,-1);
                }
                else if((ball.y()>0.55 && ball.x()>0.65) && (dist < 0.08))
                {
                    chute(id,-1);
                }else if((ball.y()<-0.55 && ball.x()>0.65) && (dist < 0.08))
                {
                    chute(id,1);
                }

            }
        }

    }else{ //Amarelo

        if((ball.x() > rb[id].x()))
        {
            int a = 0;
            //Consideração do vetor de predição
            if(abs(ball.y()-rb[id].y()) > 0.3)
                a = 1;
            else if(abs(ball.y()-rb[id].y()) < 0.06 && dist < 0.2)
            {
                (*resultante_2)[0]=rb[id].x();
                (*resultante_2)[1]=rb[id].y();
            }


            int c = 0;
            if((ball.x()/rb[id].x()<1.1 && ball.x()/rb[id].x()>1)&&abs(ball.y()-rb[id].y()) > 0.1)
                c = 1;

            double repulsivoX;
            double repulsivoY;

            if(ball.y() > rb[id].y())
            {
                repulsivoX = (c*0.1+0.1)*sin(0+(c*0.5*M_PI));
                repulsivoY = (c*0.1+0.1)*cos(0+(c*0.5*M_PI));
            }else
            {
                repulsivoX = (c*0.1+0.1)*sin(M_PI-(c*0.5*M_PI));
                repulsivoY = (c*0.1+0.1)*cos(M_PI-(c*0.5*M_PI));
            }

            //Componentes do Vetor direção
            double angle = atan2(ball.x()-predictedBall.x,ball.y()-predictedBall.y);
            componenteX = /*-c*ball.x()*/ -  a*0.1*sin(angle) - repulsivoX;
            componenteY = /*-c*ball.y()*/ -  a*0.1*cos(angle) - repulsivoY;

            (*resultante_2)[0]+= componenteX;
            (*resultante_2)[1]+= componenteY;

        }else
        {
            if(dist < 0.08)
            {
                (*resultante_2)[0]+=-gain*sin(beta+M_PI);
                (*resultante_2)[1]+=-gain*cos(beta+M_PI);
            }
            else
            {
                if((ball.x() > -0.6 && ball.y() < -0.2)&& (rb[id].y() > ball.y())/*&&(abs(ball.y()-blue[id].x())>0.07)*/)
                {
                    //Vetor de corrção
                    componenteX = /*ball.x()*/ -  0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);
                    componenteY = /*ball.y()*/ -  0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);

                    (*resultante_2)[0]+=componenteX;
                    (*resultante_2)[1]+=componenteY;
                }else if((ball.x() > -0.6 && ball.y() > 0.2)&&(rb[id].y() < ball.y())/*&&(abs(ball.y()-blue[id].x())>0.07)*/)
                {
                    //Vetor de corrção
                    componenteX = /*ball.x()*/ -  0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);
                    componenteY = /*ball.y()*/ -  0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0)/*+M_PI*/);

                    (*resultante_2)[0]+=componenteX;
                    (*resultante_2)[1]+=componenteY;

                }else if(dist > 0.2)
                {
                    (*resultante_2)[0]+= -0.1*sin(atan2(ball.x() - meioGolx,ball.y() - 0.0));
                    (*resultante_2)[1]+= -0.1*cos(atan2(ball.x() - meioGolx,ball.y() - 0.0));
                }
                //atualização
                if(((ball.y()>0.25 && ball.y()<0.6) && ball.x() < -0.6 && rb[id].x() <= ball.x() && rb[id].y()>0.25 && rb[id].y()<ball.y())
                        ||( (ball.y()<-0.25 && ball.y()>-0.6) && ball.x() < -0.6 && rb[id].x() <= ball.x() && rb[id].y()<-0.25 && rb[id].y()>ball.y()))
                {
                    (*resultante_2)[0]+= -0.1*sin(M_PI/2);
                    (*resultante_2)[1]+= -0.1*cos(M_PI/2);
                }//
            }

        }

        saturacao(resultante_2);

        if(ball.x()==0)
        {
            vaiPara(rb[id],ball.x(),ball.y(),id);
        }
        else
        {
            if(ball.x()>=0)
            {

                if(zag_dist/dist < 0.9)
                {
                    //Comportamento sem bola : Defensivo

                    /* dist_target = distância do alvo para a bola;
                     * target = alvo da marcação;
                     * (zag_dist/dist) = razão entre as distâcias do zagueiro para
                     o atacante;
                     */

                    //Calculo do alvo de marcação
                    double dist_target = 0.0;
                    int target = 0;
                    for(int player = 0; player < 3; player++)
                    {
                        double dist_adv = sqrt(pow(ball.x()-adversario[player].x(),2.0)+pow( ball.y()-adversario[player].y(),2.0));
                        if(!(adversario[player].x() > 0.65 || adversario[player].x() < -0.65) && (dist_adv > dist_target))
                        {
                            dist_target = dist_adv;
                            target = player;
                        }
                    }

                    //Pos com relação ball-alvo
                    double attachX = ball.x() - (0.8*dist_target)*sin(atan2(ball.x() - adversario[target].x(),ball.y() - adversario[target].y()));
                    double attachY = ball.y() - (0.8*dist_target)*cos(atan2(ball.x() - adversario[target].x(),ball.y() - adversario[target].y()));

                    vaiPara_desviando(rb[id],attachX,attachY,id);
                }
                else
                    vaiPara_desviando(rb[id],-0.10,(*resultante_2)[1],id);

            }else
            {
                //Comportamento Ofensivo - Posicionamento

               //Cálculo da posição
                double attachX =  ball.x() + 0.45*sin(atan2(ball.x() - rb[idzag].x(),ball.y() - rb[idzag].y()));
                //double attachY =  ball.y() + 0.45*cos(atan2(ball.x() - rb[1].x(),ball.y() - rb[1].y()));
                int c;
                if(rb[idzag].y()>0)
                    c = -1;
                else
                    c = 1;

                if(zag_dist/dist < 0.3)
                    vaiPara_desviando(rb[id],attachX-0.1,c*0.25,id);
                else
                    vaiPara_desviando(rb[id],(*resultante_2)[0],(*resultante_2)[1],id);

                if(ball.x() < -0.65 && rb[id].x() < -0.65 && dist < 0.08 && (ball.y() < 0.18 && ball.y() > -0.18)&& rb[id].y()>ball.y())
                {
                    //vaiPara(rb[id],(*resultante_2)[0],(*resultante_2)[1],id);
                    chute(id,-1);

                }else if(ball.x() < -0.65 && rb[id].x() < -0.65 && dist < 0.08 && (ball.y() < 0.18 && ball.y() > -0.18)&& rb[id].y()<ball.y())
                {
                    chute(id,1);
                }
                else if((ball.y()>0.55 && ball.x()<-0.65) && (dist < 0.08))
                {
                    chute(id,1);
                }else if((ball.y()<-0.55 && ball.x()<-0.65) && (dist < 0.08))
                {
                    chute(id,-1);
                }

            }

        }
    }
    //se tiver uma reta clara pro gol, ele vai chutar
    FIRE_KICK(rb[id],ball,id);
}


void Strategy::chute(int idRobot, int sinal)
{
    VW[idRobot][0] = 0;
    VW[idRobot][1] = 20*sinal;
}


void Strategy::goleiro_petersson2(fira_message::Robot rb,fira_message::Ball ball, int id){

    double top_limit = 0.17; //largura do gol/2
    double x_desejado = -0.7*lado;
    double delta = 0.01; // pra impedir que ele fique sambando parado no gol
    double velocidade = 2.5;
    double dist = distancia(rb,ball.x(),ball.y());
    vector <double> new_pos = {0,0};

    //se a bola estiver longe do goleiro utiliza o preditor para ajeitar sua posição
    if ( dist > 0.3){
        new_pos = {predictedBall.x,predictedBall.y};
        velocidade = 2.5;
    }else{
        new_pos = {ball.x(),ball.y()};
        velocidade = 3;
    }

    if(dist < 0.12 && ball.x() > rb.x() && rb.x() < -0.6 && ball.y() <= top_limit && ball.y() >=-top_limit && lado == 1){
        vaiPara(rb,ball.x(),ball.y(),id);
    }else if(dist < 0.12 && ball.x() < rb.x() && rb.x() > 0.6 && ball.y() <= top_limit && ball.y() >=-top_limit && lado == -1){
        vaiPara(rb,ball.x(),ball.y(),id);
    }else{
        //Verifica se o robô está perto do centro do gol
        if(distancia(rb,x_desejado,rb.y()) >= 0.02){

            if(distancia(rb,x_desejado,rb.y()) >= 0.3){
                  vaiPara_desviando(rb,x_desejado,0.0,id);
            }else{
                  vaiPara(rb,x_desejado,0.0,id);
            }

        }else{

            ang_err angulo = olhar(rb,rb.x(),top_limit + 5); // calcula diferença entre angulo atual e angulo desejado
            if(angulo.fi >= 0.5 || angulo.fi<= -0.5){ //se o robô não está aproximadamente 90 graus
                andarFrente(0,id);
                VW[id][1] = controleAngular(angulo.fi);
            }

            else if(rb.y() < top_limit && rb.y() + delta < new_pos[1]){ //robô abaixo da bola

                if(angulo.flag == 1){
                    //andarFrente(125,id);
                    VW[id][0] = velocidade;
                    VW[id][1] = 0;
                }
                else{
                    //andarFundo(125,id);
                    VW[id][0] = -velocidade;
                    VW[id][1] = 0;
                }
            }
            else if(rb.y() > -top_limit && rb.y() - delta > new_pos[1]){ //robô acima da bola
                if(angulo.flag == 1){
                    //andarFundo(125,id);
                    VW[id][0] = -velocidade;
                    VW[id][1] = 0;
                }
                else{
                    VW[id][0] = velocidade;
                    VW[id][1] = 0;
                    //andarFrente(125,id);
                }
            }
            else{
                andarFrente(0,id);
            }
            //gira se a bola estiver muito perto do goleiro
            if (distancia(rb,ball.x(),ball.y()) < 0.08){
                if (((rb.y()>0)&&(predictedBall.y>rb.y()+0.01))||((rb.y()<0)&&(predictedBall.y<rb.y()-0.01)))
                { //Só chuta se a bola não tiver a caminho de entrar no gol (condicao nova pos unball)
                    if((ball.y() < rb.y() && lado == 1)){
                       girarHorario(125,id);
                    }
                    if((ball.y() > rb.y() && lado == -1)){
                       girarHorario(125,id);
                    }
                    if((ball.y() > rb.y() && lado == 1)){
                       girarAntihorario(125,id);
                    }
                    if((ball.y() < rb.y() && lado == -1)){
                       girarAntihorario(125,id);
                    }
                }
            }
        }
    }
    double lim_x[2] = {0.58,0.75};
    double lim_y[2] = {0.17,0.38};
    //lim_x = 0.75 e 0.6 lim_y 0.2 0.35
    //azul
    if ((lado == 1) && (distancia(rb,ball.x(),ball.y()) <= 0.1) && ((rb.y()<-top_limit-0.05)||(rb.y()>top_limit+0.05))){
        if((ball.x() >= -lim_x[1]) && (ball.x() <= -lim_x[0]) && (ball.y() >= -lim_y[1]) && (ball.y() <= -lim_y[0])){
            //vaiPara(rb,ball.x(),ball.y(),id);
            if(ball.y() < rb.y()){
               girarHorario(125,id);
            }else{
               girarAntihorario(125,id);
            }
        }
        if((ball.x() >= -lim_x[1]) && (ball.x() <= -lim_x[0]) && (ball.y() <= lim_y[1]) && (ball.y() >= lim_y[0])){
                if(ball.y() < rb.y()){
                   girarHorario(125,id);
                }else{
                   girarAntihorario(125,id);
                }
        }
    //amarelo
    }else if ((lado == -1) && (distancia(rb,ball.x(),ball.y()) <= 0.1)&& ((rb.y()<-top_limit-0.05)||(rb.y()>top_limit+0.05))){
        if((ball.x() <= lim_x[1]) && (ball.x() >= lim_x[0]) && (ball.y() >= -lim_y[1]) && (ball.y() <= -lim_y[0])){
            //vaiPara(rb,ball.x(),ball.y(),id);
            //vaiPara(rb,ball.x(),ball.y(),id);
            if(ball.y() > rb.y()){
               girarHorario(125,id);
            }else{
               girarAntihorario(125,id);
            }
        }
        if((ball.x() <= lim_x[1]) && (ball.x() >= lim_x[0]) && (ball.y() <= lim_y[1]) && (ball.y() >= lim_y[0])){
           // vaiPara(rb,ball.x(),ball.y(),id);
            if(ball.y() > rb.y()){
               girarHorario(125,id);
            }else{
               girarAntihorario(125,id);
            }
        }
    }

}

void Strategy::FIRE_KICK(fira_message::Robot rb,fira_message::Ball ball, int id){

    double lim_x = 0.8; //Posicao x do centro do gol
    double lim_y = 0.17; //Define a localização do gol em y
    double distancia_posse = 0.1; //Distância que o robô considera que ele tem posse da bola
    double lim_ang = 0.1; //Pra quando a bola estiver longe
    double lim_ang_perto = 0.78; //Pra quando a bola estiver perto
    double dist_ball_rb[2] = {ball.x() - rb.x(),ball.y() - rb.y()}; //0 é x, 1 é y
    double dist_gol_rb[2] = {lado*lim_x - rb.x(),lim_y - rb.y()};
    double Y = dist_gol_rb[0]*tan(rb.orientation()) + rb.y();
    double alvo_projetado[2] = {lim_x,Y};
    //posicao alvo: [lim_x, Y] -> a projeção de onde o robô tá olhando

    double ang_rb_bola = atan2(dist_ball_rb[1],dist_ball_rb[0]);
    ang_err angulo = olhar(rb, ball.x(), ball.y()); //Angulo pra bola
    ang_err angulo_alvo = olhar(rb,alvo_projetado[0],alvo_projetado[1]);

    bool flag = true;
    bool flag_lateral = false;
    flag_lateral = (((lado==-1)&&(rb.x()>ball.x()))||((lado==1)&&(rb.x()<ball.x())));
    double x = distancia(rb,ball.x(),ball.y());

    lim_ang = cos(x)*atan2(sqrt(0.04*(1-(x*x/4))),x); //elipse era 0.06
    //lim_ang = 1.57-1.47*x;
    //cout << "ID: "<< id << "lim_ang: "<<lim_ang <<endl;

    /*if (distancia(rb,ball.x(),ball.y()) < distancia_posse) {
        //Se a bola tiver muito perto, é importante aumentar o angulo
        lim_ang = lim_ang_perto;
    }*/

    /*cout << "Orientacao: " << rb.orientation() << "Angulo pra bola: " << angulo.fi << endl;
    cout << "Y de projecao: " << Y << endl;*/

    //Se o robô estiver olhando pro gol
    if (Y >= -lim_y && Y<= lim_y){
        //cout<<"Olhando pro gol"<<endl;
        //Se o robo estiver olhando pra bola
        if( (flag_lateral) && (abs(angulo.fi*M_PI/180) <= lim_ang)){
            cout<<"FIRE!"<<endl;
            VW[id][0] = angulo.flag*5;
            VW[id][1] = 0;
            bandeira = false;
            flag = false;
        }else{
            cout<<"Olhando pro gol mas não pra bola"<<endl;
            bandeira = true;
        }
    }else{
        cout<<"Não estou olhando pro gol"<<endl;
        bandeira = true;
    }
    if(flag){
        //andarFrente(0,id);
    }

}

vector<pair<double, double> > Strategy::getWayPoints()
{
    return caminho;
    //teste 2
}

void Strategy::filtro_caminho(double frequencia)
{
  vector<vetor>caminho_filtrado;
  caminho_filtrado.clear();
  caminho_filtrado.push_back(caminho.at(0));
  double a = frequencia; //mantenha: 0 < frequência < 1
  double px;
  double py;
  vetor p;
  for (int i = 1; i< (int) caminho.size(); i++ ){
    px = (1 - a)*caminho_filtrado.at(i-1).first + a*caminho.at(i).first;
    py = (1 - a)*caminho_filtrado.at(i-1).second + a*caminho.at(i).second;
    p = make_pair(px,py);
    caminho_filtrado.push_back(p);
  }
  caminho_filtrado.push_back(*caminho.end());
  caminho = caminho_filtrado;
}

