#include <QCoreApplication>

#include <thread>
#include <utils/timer/timer.h>
#include <clients/vision/visionclient.h>
#include <clients/referee/refereeclient.h>
#include <clients/actuator/actuatorclient.h>
#include <clients/replacer/replacerclient.h>

#include <strategy.h>
#include <string.h>
#include <QFile>
#include <iostream>
#include <sstream>
#include <iomanip>


QString getFoulNameById(VSSRef::Foul foul){
    switch(foul){
        case VSSRef::Foul::FREE_BALL:    return "FREE_BALL";
        case VSSRef::Foul::FREE_KICK:    return "FREE_KICK";
        case VSSRef::Foul::GOAL_KICK:    return "GOAL_KICK";
        case VSSRef::Foul::PENALTY_KICK: return "PENALTY_KICK";
        case VSSRef::Foul::KICKOFF:      return "KICKOFF";
        case VSSRef::Foul::STOP:         return "STOP";
        case VSSRef::Foul::GAME_ON:      return "GAME_ON";
        default:                         return "FOUL NOT IDENTIFIED";
    }
}

QString getTeamColorNameById(VSSRef::Color color){
    switch(color){
        case VSSRef::Color::NONE:    return "NONE";
        case VSSRef::Color::BLUE:    return "BLUE";
        case VSSRef::Color::YELLOW:  return "YELLOW";
        default:                     return "COLOR NOT IDENTIFIED";
    }
}

QString getQuadrantNameById(VSSRef::Quadrant quadrant){
    switch(quadrant){
        case VSSRef::Quadrant::NO_QUADRANT: return "NO QUADRANT";
        case VSSRef::Quadrant::QUADRANT_1:  return "QUADRANT 1";
        case VSSRef::Quadrant::QUADRANT_2:  return "QUADRANT 2";
        case VSSRef::Quadrant::QUADRANT_3:  return "QUADRANT 3";
        case VSSRef::Quadrant::QUADRANT_4:  return "QUADRANT 4";
        default:                            return "QUADRANT NOT IDENTIFIED";
    }
}

QString getHalfNameById(VSSRef::Half half){
    switch(half){
        case VSSRef::Half::NO_HALF: return "NO_HALF";
        case VSSRef::Half::FIRST_HALF: return "FIRST HALF";
        case VSSRef::Half::SECOND_HALF: return "SECOND HALF";
        default: "NO HALF DEFINED";
    }
}

void reposicionar_azul(double x[],double y[],double ori[],ReplacerClient *replacerClient){
    // First creating an placement command for the blue team
    //VSSRef::team_to_ref::VSSRef_Placement placementCommandBlue;
    //VSSRef::Frame *placementFrameBlue = new VSSRef::Frame();
    //placementFrameBlue->set_teamcolor(VSSRef::Color::BLUE);

    for(int i = 0; i < 3; i++){
        //VSSRef::Robot *robot = placementFrameBlue->add_robots();
        //robot->set_robot_id(static_cast<uint32_t>(i));
        //robot->set_x(x[i]);
        //robot->set_y(y[i]);
        //robot->set_orientation(ori[i]);
        replacerClient->placeRobot(i,x[i], y[i], ori[i]);
    }

    //placementCommandBlue.set_allocated_world(placementFrameBlue);
    replacerClient->sendFrame();
    // Sending blue
    //std::string msgBlue;
    //placementCommandBlue.SerializeToString(&msgBlue);
    //if(replacerSocket->write(msgBlue.c_str(), static_cast<qint64>(msgBlue.length())) == -1){
    //    std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
    //}
}

void reposicionar_amarelo(double x[],double y[],double ori[],ReplacerClient *replacerClient){
    // Now creating an placement command for the yellow team
    //VSSRef::team_to_ref::VSSRef_Placement placementCommandYellow;
    //VSSRef::Frame *placementFrameYellow = new VSSRef::Frame();
    //placementFrameYellow->set_teamcolor(VSSRef::Color::YELLOW);

    for (int i = 0; i<3 ;i++){
        //VSSRef::Robot *robot = placementFrameYellow->add_robots();
        //robot->set_robot_id(static_cast<uint32_t>(i));
        //robot->set_x(x[i]);
        //robot->set_y(y[i]);
        //robot->set_orientation(ori[i]);
        replacerClient->placeRobot(i,x[i], y[i], ori[i]);
    }
    replacerClient->sendFrame();


    //placementCommandYellow.set_allocated_world(placementFrameYellow);

    // Sending yellow
    //std::string msgYellow;
    //placementCommandYellow.SerializeToString(&msgYellow);
    //if(replacerSocket->write(msgYellow.c_str(), static_cast<qint64>(msgYellow.length())) == -1){
    //    std::cout << "[Example] Failed to write to replacer socket: " << replacerSocket->errorString().toStdString() << std::endl;
    // }
}



int main(int argc, char *argv[]) {
    //QCoreApplication a(argc, argv);

    //SETUP VISAO
    string IP;
    string vision;
    string command;
    string referee_port;
    string replacer_port;
    string campo;
    //IP="224.0.0.1";
    //vision="10002";
    //command="20011";
    //referee_port="10003";
    //replacer_port="10004";
    //campo="azul";

    if(argc == 7){ // se existir entrada via terminal
            IP = argv[1];
            vision = argv[2];
            command = argv[3];
            referee_port = argv[4];
            replacer_port = argv[5];
            campo = argv[6];
            std::cout << campo + '\n';
    }else{         //se não existir executa os valores abaixo
        IP = "224.0.0.1";
        vision = "10002";
        command = "20011";
        referee_port = "10003";
        replacer_port = "10004";
        campo = "amarelo";
        std::cout << campo + '\n';
    }
    //converte de string pra inteiro
    stringstream aux(vision);
    int visao = 0;
    aux >> visao;
    //converte de string pra inteiro
    stringstream aux2(command);
    int comando = 0;
    aux2 >> comando;
    //converte de string pra inteiro
    stringstream aux3(referee_port);
    int referee_porta = 0;
    aux3 >> referee_port;
    //converte de string pra inteiro
    stringstream aux4(replacer_port);
    int replacer_porta = 0;
    aux4 >> replacer_port;


    // Starting timer
    Timer timer;

    // Creating client pointers
    VisionClient *visionClient = new VisionClient("224.0.0.1", visao);
    RefereeClient *refereeClient = new RefereeClient("224.5.23.2", referee_porta);
    ReplacerClient *replacerClient = new ReplacerClient("224.5.23.2", replacer_porta);
    ActuatorClient *actuatorClient = new ActuatorClient("127.0.0.1", comando);

                                                                                                                                            // Setting our color as BLUE at left side
                                                                                                                                            //VSSRef::Color ourColor = VSSRef::Color::YELLOW;
                                                                                                                                            //bool ourSideIsLeft = false;

    VSSRef::Color ourColor;
    bool ourSideIsLeft;
    bool my_robots_are_yellow = false;
    bool my_robots_are_blue = false;
    bool ambos;
    if (campo == "azul"){
        ourColor = VSSRef::Color::BLUE;
        ourSideIsLeft = true;
        my_robots_are_yellow = false;
        my_robots_are_blue = true;
        ambos = false;
    }else if (campo == "amarelo"){
        ourColor = VSSRef::Color::YELLOW;
        ourSideIsLeft = false;
        my_robots_are_yellow = true;
        my_robots_are_blue = false;
        ambos = false;
    //}else if (campo == "ambos"){
    //    my_robots_are_yellow = true;
    //    my_robots_are_blue = true;
    //    ambos = true;
    }else{
            printf("ERRO ESCOLHA DE CAMPO");
            exit(EXIT_FAILURE);
        }

    // Desired frequency (in hz)
    float freq = 60.0;

    // Setting actuator and replacer teamColor
    actuatorClient->setTeamColor(ourColor);
    replacerClient->setTeamColor(ourColor);

    //inicialização da classe estratégia

    //inicializa duas estrategias uma para cada time em caso de ambos os times serem utilizados
     Strategy estrategia_amarela(my_robots_are_yellow);
     Strategy estrategia_azul(my_robots_are_blue);

     //inicializa estrategia azul ou amarela
     Strategy estrategia(my_robots_are_yellow);

     string ultimo_comando;
     string cor;
     string quadrante;
     //Lendo o JUIZ
     VSSRef::ref_to_team::VSSRef_Command send_command;


    while(1) {
        // Start timer
        timer.start();

        // Running vision and referee clients
        visionClient->run();
        refereeClient->run();


        ultimo_comando = getFoulNameById(send_command.foul()).toStdString();
        cor = getTeamColorNameById(send_command.teamcolor()).toStdString();
        quadrante = getQuadrantNameById(send_command.foulquadrant()).toStdString();

        cout<<ultimo_comando<<endl<<cor<<endl<<quadrante<<endl;

       //ENVIA REPOSICIONAMENTO AMARELO
       if(my_robots_are_yellow){

           if(ultimo_comando == "FREE_BALL"){
                 double x[3] = {0.7 , 0.3 , -0.3};
                 double y[3] = {0 , 0 , 0.3};
                 double ori[3] = {90 , 0 , 0};

                 if(quadrante == "QUADRANT 1"){
                     x[2] = 0.59;
                     y[2] = 0.4;
                     ori[2] = 185;
                 }
                 if(quadrante == "QUADRANT 4"){
                     x[2] = 0.59;
                     y[2] = -0.4;
                     ori[2] = 175;
                 }
                 if(quadrante == "QUADRANT 2"){
                     x[2] = -0.18;
                     y[2] = 0.4;
                 }
                 if(quadrante == "QUADRANT 3"){
                     x[2] = -0.18;
                     y[2] = -0.4;
                 }
                 reposicionar_amarelo(x,y,ori,replacerClient);
           }

           if(ultimo_comando == "KICKOFF"){
                if(cor == "YELLOW"){
                    double x[3] = {0.7,0.3,0.05};
                    double y[3] = {0,0,0.05};
                    double ori[3] = {90,0,-135};
                    reposicionar_amarelo(x,y,ori,replacerClient);
                }
                if(cor == "BLUE"){
                    double x[3] = {0.7,0.3,0.5};
                    double y[3] = {0,0,0};
                    double ori[3] = {90,0,0};
                    reposicionar_amarelo(x,y,ori,replacerClient);
                }
           }

           if(ultimo_comando == "GOAL_KICK"){
               //posição da bola = 0.6,0.35
               //posição goleiro = 0.65, 0.3
               if(cor == "BLUE"){
                   double x[3] = {0.7,-0.2,-0.2};
                   double y[3] = {0,-0.4,0.4};
                   double ori[3] = {90,0,0};
                   reposicionar_amarelo(x,y,ori,replacerClient);
               }
               if(cor == "YELLOW"){
                   double x[3] = {0.7,0.67,0.67};
                   double y[3] = {0,-0.37,0.37};
                   double ori[3] = {90,165,195};
                   reposicionar_amarelo(x,y,ori,replacerClient);

               }
           }

           if(ultimo_comando == "PENALTY_KICK"){
               double gol[4] = {25,15,-15,-25};
               int sorteio = rand()%4 ;
               double orientacao = gol[sorteio];
               cout<<sorteio<<"  "<<orientacao<<endl;
               if(cor == "BLUE"){
                   double x[3] = {0.7,-0.05,-0.05};
                   double y[3] = {0,-0.3,0.3};
                   double ori[3] = {90,0,0};
                   reposicionar_amarelo(x,y,ori,replacerClient);
               }
               if(cor == "YELLOW"){
                   double x[3] = {0.7,0.05,-0.3};
                   double y[3] = {0,0,0};
                   double ori[3] = {90,0,orientacao};
                   reposicionar_amarelo(x,y,ori,replacerClient);
               }
           }
        }
         //ENVIA REPOSICIONAMENTO AZUL
        if(my_robots_are_blue){ //AZUL
             if(ultimo_comando == "FREE_BALL"){
                   double x[3] = {-0.7,-0.3,-0.3};
                   double y[3] = {0,0,0.3};
                   double ori[3] = {90,0,0};

                   if(quadrante == "QUADRANT 1"){
                       x[2] = 0.18;
                       y[2] = 0.4;
                   }
                   if(quadrante == "QUADRANT 4"){
                       x[2] = 0.18;
                       y[2] = -0.4;
                   }
                   if(quadrante == "QUADRANT 2"){
                       x[2] = -0.59;
                       y[2] = 0.4;
                       ori[2] = -5;
                   }
                   if(quadrante == "QUADRANT 3"){
                       x[2] = -0.59;
                       y[2] = -0.4;
                       ori[2] = 5;
                   }
                   reposicionar_azul(x,y,ori,replacerClient);
           }

           if(ultimo_comando == "KICKOFF"){
               if(cor == "BLUE"){
                   double x[3] = {-0.7,-0.3,-0.05};
                   double y[3] = {0,0,0.05};
                   double ori[3] = {90,0,-45};
                   reposicionar_azul(x,y,ori,replacerClient);
               }
               if(cor == "YELLOW"){
                   double x[3] = {-0.7,-0.3,-0.5};
                   double y[3] = {0,0,0};
                   double ori[3] = {90,0,0};
                   reposicionar_azul(x,y,ori,replacerClient);
               }
           }

           if(ultimo_comando == "GOAL_KICK"){
               //posição da bola = 0.6,0.35
               //posição goleiro = 0.65, 0.3

               if(cor == "BLUE"){
                   double x[3] = {-0.7,-0.67,-0.67};
                   double y[3] = {0,-0.37,0.37};
                   double ori[3] = {90,195,165};
                   reposicionar_azul(x,y,ori,replacerClient);

               }
               if(cor == "YELLOW"){
                   double x[3] = {-0.7,0.2,0.2};
                   double y[3] = {0,-0.4,0.4};
                   double ori[3] = {90,0,0};
                   reposicionar_azul(x,y,ori,replacerClient);
               }
           }

           if(ultimo_comando == "PENALTY_KICK"){
               double gol[4] = {25,15,-15,-25};
               int sorteio = rand()%4 ;
               double orientacao = gol[sorteio];
               cout<<sorteio<<"  "<<orientacao<<endl;
               if(cor == "BLUE"){
                   double x[3] = {-0.7,-0.05,0.3};
                   double y[3] = {0,0,0};
                   double ori[3] = {90,0,orientacao};
                   reposicionar_azul(x,y,ori,replacerClient);
               }
               if(cor == "YELLOW"){
                   double x[3] = {-0.7,0.05,0.05};
                   double y[3] = {0,-0.3,0.3};
                   double ori[3] = {90,0,0};
                   reposicionar_azul(x,y,ori,replacerClient);
               }
           }
       }


        // Debugging vision
        fira_message::sim_to_ref::Environment lastEnv = visionClient->getLastEnvironment();
        if(lastEnv.has_frame()) {
            // Taking last frame
            fira_message::Frame lastFrame = lastEnv.frame();
            /*
            // Debugging ball
            std::cout << "\n===== BALL =====\n";
            QString ballDebugStr = QString("Ball x: %1 y: %2")
                                .arg(lastFrame.ball().x())
                                .arg(lastFrame.ball().y());
            std::cout << ballDebugStr.toStdString() + '\n';

            // Debugging blue robots
            std::cout << "\n===== BLUE TEAM =====\n";
            for(int i = 0; i < lastFrame.robots_blue_size(); i++) {
                QString robotDebugStr = QString("Robot %1 -> x: %2 y: %3 ori: %4")
                        .arg(lastFrame.robots_blue(i).robot_id())
                        .arg(lastFrame.robots_blue(i).x())
                        .arg(lastFrame.robots_blue(i).y())
                        .arg(lastFrame.robots_blue(i).orientation());
                std::cout << robotDebugStr.toStdString() + '\n';
            }

            // Debugging yellow robots
            std::cout << "\n===== YELLOW TEAM =====\n";
            for(int i = 0; i < lastFrame.robots_yellow_size(); i++) {
                QString robotDebugStr = QString("Robot %1 -> x: %2 y: %3 ori: %4")
                        .arg(lastFrame.robots_yellow(i).robot_id())
                        .arg(lastFrame.robots_yellow(i).x())
                        .arg(lastFrame.robots_yellow(i).y())
                        .arg(lastFrame.robots_yellow(i).orientation());
                std::cout << robotDebugStr.toStdString() + '\n';
            }
            */
            int robots_blue_n =  lastFrame.robots_blue_size();
            int robots_yellow_n =  lastFrame.robots_yellow_size();
            //Ball info:
            fira_message::Ball ball = lastFrame.ball();

            fira_message::Robot b0 = lastFrame.robots_blue(0);
            fira_message::Robot b1 = lastFrame.robots_blue(1);
            fira_message::Robot b2 = lastFrame.robots_blue(2);
            //Yellow
            fira_message::Robot y0 = lastFrame.robots_yellow(0);
            fira_message::Robot y1 = lastFrame.robots_yellow(1);
            fira_message::Robot y2 = lastFrame.robots_yellow(2);

            if (ambos){
                estrategia_azul.predict_ball(ball);
                estrategia_amarela.predict_ball(ball);
            }else{
                estrategia.predict_ball(ball);
            }

            //printf("-[Geometry Data]-------\n");
            const fira_message::Field & field = lastEnv.field();

            if(ambos){
                estrategia_amarela.atualiza_pos(b0,b1,b2,y0,y1,y2);
                estrategia_azul.atualiza_pos(b0,b1,b2,y0,y1,y2);

                estrategia_amarela.strategy_yellow(y0,y1,y2,b0,b1,b2,ball,field,"ultimo_comando");
                estrategia_azul.strategy_blue(b0,b1,b2,y0,y1,y2,ball,field,"ultimo_comando");

                //Enviando velocidades para os robos amarelos
                for(int i = 0;i < estrategia_amarela.qtdRobos;i++)
                    //commandClient.sendCommand(estrategia_amarela.vRL[i][1],estrategia_amarela.vRL[i][0],my_robots_are_yellow,i);
                    actuatorClient->sendCommand(i, estrategia_amarela.vRL[i][1], estrategia_amarela.vRL[i][0]);

                //Enviando velocidades para os robos azuis
                for(int i = 0;i < estrategia_azul.qtdRobos;i++)
                    //commandClient.sendCommand(estrategia_azul.vRL[i][1],estrategia_azul.vRL[i][0],!my_robots_are_yellow,i);
                    actuatorClient->sendCommand(i, estrategia_azul.vRL[i][1], estrategia_azul.vRL[i][0]);

            }else{
                estrategia.atualiza_pos(b0,b1,b2,y0,y1,y2);

                if(my_robots_are_yellow){
                    estrategia.strategy_yellow(y0,y1,y2,b0,b1,b2,ball,field,"ultimo_comando");
                }else{
                    estrategia.strategy_blue(b0,b1,b2,y0,y1,y2,ball,field,"ultimo_comando");
                }
                //Enviando velocidades para os robos
                for(int i = 0;i < estrategia.qtdRobos;i++)
                    actuatorClient->sendCommand(i,estrategia.vRL[i][1],estrategia.vRL[i][0]);
                    //actuatorClient->sendCommand(i, estrategia_amarela.vRL[i][1], estrategia_amarela.vRL[i][0]);


            }

        }

        // Sending robot commands for robot 0, 1 and 2
        //actuatorClient->sendCommand(0, 0, 0);
        //actuatorClient->sendCommand(1, 0, 0);
        //actuatorClient->sendCommand(2, 0, 0);



        //REFEREE FLAGS
        /*
            GAME_ON
            FREE_BALL
            FREE_KICK *
            GOAL_KICK
            PENALTY_KICK
            KICKOFF
            STOP *
            HALT *
            Foul_INT_MAX_SENTINEL_DO_NOT_USE_
            Foul_INT_MIN_SENTINEL_DO_NOT_USE_
        */

        //ENVIA REPOSICIONAMENTO AMARELO
       if(my_robots_are_yellow){

       }
       //ENVIA REPOSICIONAMENTO AZUL
      if(my_robots_are_blue){ //AZUL

      }
        // If is kickoff, send this test frame!
      if(refereeClient->getLastFoul() == VSSRef::Foul::FREE_BALL) {
          replacerClient->placeRobot(0, ourSideIsLeft ? -0.2 : 0.2, 0, 0);
          replacerClient->placeRobot(1, ourSideIsLeft ? -0.2 : 0.2, 0.2, 0);
          replacerClient->placeRobot(2, ourSideIsLeft ? -0.2 : 0.2, -0.2, 0);
          replacerClient->sendFrame();
      }

        // Stop timer
        timer.stop();

        // Sleep for remainingTime
        long remainingTime = (1000 / freq) - timer.getMiliSeconds();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    // Closing clients
    visionClient->close();
    refereeClient->close();
    replacerClient->close();
    actuatorClient->close();

    //return a.exec();
    return 0;
}
