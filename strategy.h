#ifndef STRATEGY_H
#define STRATEGY_H
#include <QtNetwork>
#include <stdio.h>
//#include "net/robocup_ssl_client.h"
//#include "net/grSim_client.h"


#include "proto/command.pb.h"
#include "proto/common.pb.h"
#include "proto/packet.pb.h"
#include "proto/replacement.pb.h"
#include "proto/vssref_command.pb.h"
#include "proto/vssref_common.pb.h"
#include "proto/vssref_placement.pb.h"
#include <time.h>



#include <utils/timer/timer.h>
#include <clients/vision/visionclient.h>
#include <clients/referee/refereeclient.h>
#include <clients/actuator/actuatorclient.h>
#include <clients/replacer/replacerclient.h>


using namespace std;

typedef pair<double, double> vetor;


struct ang_err
{
    double fi;
    int flag;
};


//Estrutura para simplificar o uso do preditor
//da bola.
struct ballPredPos
{
    double x;
    double y;
};

class Team : public std::array<fira_message::Robot,6>
{
public:


    // the constructors
    Team() {}
    Team(fira_message::Robot rb0, fira_message::Robot rb1, fira_message::Robot rb2)
    {

        (*this)[0] = rb0;
        (*this)[1] = rb1;
        (*this)[2] = rb2;

    }
    Team(fira_message::Robot rb0, fira_message::Robot rb1, fira_message::Robot rb2,
         fira_message::Robot rb3, fira_message::Robot rb4, fira_message::Robot rb5)
    {

        (*this)[0] = rb0;
        (*this)[1] = rb1;
        (*this)[2] = rb2;
        (*this)[3] = rb3;
        (*this)[4] = rb4;
        (*this)[5] = rb5;

    }


    //Destructors
    ~Team(){}

};

class Strategy {
  public:

    Strategy(bool time);
    ~Strategy();

    Team blue;
    Team yellow;
    Team robots;

    vector<ballPredPos> ballPredMemory; //Vetor de memória com posições passadas
    void predict_ball(fira_message::Ball ball);
    ballPredPos predictedBall; //Inicializado no construtor

    vector<vector<double>> vRL, VW;
    vector<pair<double,double>> caminho;
    int detectChange;

    int qtdRobos, vrMax;
    double Vmax, Wmax;

    int lado;

    bool bandeira = true;

    void strategy_blue(fira_message::Robot b0, fira_message::Robot b1, fira_message::Robot b2,
                      fira_message::Robot y0, fira_message::Robot y1, fira_message::Robot y2
                     , fira_message::Ball ball, const fira_message::Field & field, int);

    void strategy_yellow(fira_message::Robot y0, fira_message::Robot y1, fira_message::Robot y2,
                         fira_message::Robot b0, fira_message::Robot b1, fira_message::Robot b2,
                         fira_message::Ball ball, const fira_message::Field & field, int);

    void girarHorario(double,int);
    void girarAntihorario(double,int);
    void andarFrente(double,int);
    void andarFundo(double,int);
    void vaiPara(fira_message::Robot,double,double,int);

    void vaiParaDinamico(fira_message::Robot,double,double,int);
    void vaiParaDinamico2(fira_message::Robot,double,double,int);
    double controleAngular(double);
    double controleLinear(fira_message::Robot,double,double,double);

    //Alterações Petersson
    double pos_robos[6][2];
    double velocidades[6];
    double velocidades_azul[3][2];
    double velocidades_amarelo[3][2];

    void saturacao(double V[]);
    void saturacao(vector <double> *V); //Overload
    void atualiza_pos(fira_message::Robot b0,fira_message::Robot b1,fira_message::Robot b2,fira_message::Robot y0,fira_message::Robot y1,fira_message::Robot y2);
    void calc_repulsao(fira_message::Robot rb, double F[]);
    void converte_vetor(double V[],double);
    double filtro(double V,int);
    void vaiPara_desviando(fira_message::Robot,double,double,int);
    void goleiro_petersson(fira_message::Robot,fira_message::Ball,int);
    void goleiro_petersson2(fira_message::Robot,fira_message::Ball,int);
    void Goleiro_linha(fira_message::Robot rb, fira_message::Ball ball, int id);
    void chute(int);
    void zagueiro2(fira_message::Robot rb, fira_message::Ball ball, int id);
    void calc_repulsao2(fira_message::Robot rb,double F[]);
    void atacante_todos(Team my,Team adv, fira_message::Ball ball, int id, int idzag);
    void chute(int idRobot, int sinal);
    void vaiPara_desviando2(fira_message::Robot,double,double,int);
    void FIRE_KICK(fira_message::Robot rb,fira_message::Ball ball, int id);
    void FIRE_KICK(fira_message::Robot rb,pair<double,double> ball, int id);



    //Atributos para zagueiro_cone
     vector<pair<double,double>>* componentes = NULL;

    //Atributos para atacante_coneLaam
    vector<pair<double,double>>* componentes_2 = NULL;
    vector<double>* resultante_2 = NULL;
    vector<string>* name_vectors = NULL;


    //Métodos para interface
    vector<pair<double,double>> getWayPoints();

  private:
    double L; //Distância entre roda e centro
    double R; //Raio da roda
    void cinematica_azul(); //transforma V e W em Vr e Vl do time azul
    void cinematica_amarelo(); //transforma V e W em Vr e Vl do time amarelo

    void atualiza_memoria_azul(double, double);
    vector<double> memoria_azul_linear;
    vector<double> memoria_azul_angular;
    void atualiza_memoria_amarelo(double, double);
    vector<double> memoria_amarelo_linear;
    vector<double> memoria_amarelo_angular;

    ang_err olhar(fira_message::Robot, double, double); //<-- usamos essa
    double distancia(fira_message::Robot,double,double);
    double distancia(double,double,double,double); //<-- usamos essa
    double limita_velocidade(double, double); //<-- usamos essa

  public:
//------------------ Edições 2021 ------------------------------------------------------------------------------------------------//
 //Métodos
   //----------- Path Planning ------------------
    vector<pair<double,double>> gerar_caminho(int qtd_pontos);
    vector<pair<double,double>> path_planningRRT(pair<double,double> root,pair<double,double> goal,Team adv, int idxRobo, int numInt, double SearchRaio);
        //-------------- RRT -----------------
        pair<double,double> gerarPontoAleatotio(pair<double,double> currPos,double raio);
        pair<double,double> gerarPontoAleatotio(pair<double,double> currPos,pair<double,double> goal,double prob,double raio);     //Sobrecarga
        int buscaPontoMaisProximo(vector<pair<double,double>> arvore, pair<double,double> q_rand);
        pair<double,double> gerarNovoPonto(pair<double,double> q_near,pair<double,double> q_rand, double passo);  /*gera o q_new*/
        bool regraDeExclusao(pair<double,double> q_new /* novo ponto */, Team obs /* outros robôs */, int idx /* indice do robô */);
        vector<pair<double,double>> gerarCaminho(vector<pair<double,double>> arvore,vector<int> adjList, int idxGoal);
        bool straitlineTest(pair<double,double> currPos/* novo ponto */, pair<double,double> target, Team obs /* outros robôs */, int idx /* indice do robô */);
        void filtro_caminho(double frequencia);


        //retorna um booleano "true" caso a bola "desvie muito" do caminho
        bool dotCriterio(pair<double,double> currPos, pair<double,double> carrotP, pair<double,double> goalP);

        //------------------------------------
    //--------------------------------------------

    //----- Comportamentos
        //ir para ponto
    void vaiParaRRT(fira_message::Robot robot,int id_robot, pair<double,double> goalP,pair<double, double> bola);
        //Tenatar levar a bola para o goal
    void takeBallToGoal(fira_message::Robot robot,int id_robot, pair<double,double> ballPos);
        int flagTbT; //Flag
        double posx,posy,vx,vy,dx,dy,ux,uy; //Variáveis para o takeBallToGoal
        //Atacante monstro
        void Cr7(fira_message::Robot rb, fira_message::Ball ball, int id);

       //erro de orientação ao ponto de interesse até a bola
    ang_err turnToDestination(fira_message::Robot robot, pair<double, double> ball, pair<double, double> dest);
       //Comportamento pro penalty
    void PENALTY_KICK(int, ang_err);

   //------ Path Tracking
    void pure_pursuit(fira_message::Robot robot,int id_robot,vector<pair<double,double>> points, double lookAhead_dist, double v_pref);
    void pure_pursuit2(fira_message::Robot robot,int id_robot,vector<pair<double,double>> points, ang_err err_to_goal);
    pair<double,double> sweep_path(pair<double,double> goal_p, pair<double,double> ref_p, double v_pref);
    double controleLinear(double err, double Kp, double Ki);
    void setup_pure_pursuit(pair<double,double> p);
         //Obs.: O controle angular e linear será o mesmo do VaiPara



 //Atributos para o path tracking
    int err_i; //memória para o integrador do controleLinear - 2
    int pivot_pp;
    pair<double,double> carrot_point;
    double Mdist;  //Distância percorrida pelo robô

 //Atributos para o path planning
    bool replain; //Flag para o replanejamento
    bool plain;


    //exportar dados
    void send_data_robots(Team blue_or_yellow, QString name="robot_data.txt");
    void send_data_ball(double x, double y,  QString name="ball_data.txt");
    void send_data_path(vector<pair<double,double>> pontos, QString name="path_data.txt" );
    void send_data_control(double setPoint, double varControl, QString name="control_data.txt" );

  //Variáveis para medição de tempo
    clock_t start,end;
    double tempo;



};



#endif // STRATEGY_H

