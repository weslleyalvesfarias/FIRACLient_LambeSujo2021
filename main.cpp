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



void reposicionar_time(double x[],double y[],double ori[],ReplacerClient *replacerClient){
    for(int i = 0; i < 3; i++){
        replacerClient->placeRobot(i,x[i], y[i], ori[i]);
    }
    replacerClient->sendFrame();
}



int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    //SETUP VISAO
    QString IP;
    QString vision;
    QString command;
    QString referee_port;
    QString replacer_port;
    QString campo;

    if(argc == 7){ // se existir entrada via terminal
        IP = argv[1];
        vision = argv[2];
        command = argv[3];
        referee_port = argv[4];
        replacer_port = argv[5];
        campo = argv[6];
        //std::cout << campo.toStdString() + '\n';
    }else{         //se não existir executa os valores abaixo
        IP = "224.0.0.1";
        vision = "10002";
        command = "20011";
        referee_port = "10003";
        replacer_port = "10004";
        campo = "amarelo";
        //std::cout << campo +" default" + '\n';
        }

    // Starting timer
    Timer timer;

    // Creating client pointers
    VisionClient *visionClient = new VisionClient("224.0.0.1", vision.toInt());
    RefereeClient *refereeClient = new RefereeClient("224.5.23.2", referee_port.toInt());
    ReplacerClient *replacerClient = new ReplacerClient("224.5.23.2", replacer_port.toInt());
    ActuatorClient *actuatorClient = new ActuatorClient("127.0.0.1", command.toInt());




    // Setting our color as BLUE at left side
    VSSRef::Color ourColor;
    bool ourSideIsLeft;

    if (campo == "azul"){
        ourColor = VSSRef::Color::BLUE;
        ourSideIsLeft = true;
    }else if (campo == "amarelo"){
        ourColor = VSSRef::Color::YELLOW;
        ourSideIsLeft = false;
    }else{
            printf("ERRO ESCOLHA DE CAMPO");
            exit(EXIT_FAILURE);
    }


    // Desired frequency (in hz)
    float freq = 60.0;

    // Setting actuator and replacer teamColor
    actuatorClient->setTeamColor(ourColor);
    replacerClient->setTeamColor(ourColor);


    //inicializa estrategia azul ou amarela
    Strategy estrategia(!ourSideIsLeft);



    while(1) {
        // Start timer
        timer.start();

        // Running vision and referee clients
        visionClient->run();
        refereeClient->run();

        // Reposition
        //ENVIA REPOSICIONAMENTO AMARELO
       if(!ourSideIsLeft){

           if(refereeClient->getLastFoul() == VSSRef::Foul::FREE_BALL){
                 double x[3] = {0.7 , 0.3 , -0.3};
                 double y[3] = {0 , 0 , 0.3};
                 double ori[3] = {90 , 0 , 0};

                 if(refereeClient->getLastFoulQuadrant() == VSSRef::Quadrant::QUADRANT_1){
                     x[2] = 0.59;
                     y[2] = 0.4;
                     ori[2] = 185;
                 }
                 if(refereeClient->getLastFoulQuadrant() == VSSRef::Quadrant::QUADRANT_4){
                     x[2] = 0.59;
                     y[2] = -0.4;
                     ori[2] = 175;
                 }
                 if(refereeClient->getLastFoulQuadrant() == VSSRef::Quadrant::QUADRANT_2){
                     x[2] = -0.18;
                     y[2] = 0.4;
                 }
                 if(refereeClient->getLastFoulQuadrant() == VSSRef::Quadrant::QUADRANT_3){
                     x[2] = -0.18;
                     y[2] = -0.4;
                 }
                 reposicionar_time(x,y,ori,replacerClient);
           }

           if(refereeClient->getLastFoul() == VSSRef::Foul::KICKOFF){
                if(refereeClient->getLastFoulColor() == VSSRef::Color::YELLOW){
                    double x[3] = {0.7,0.3,0.05};
                    double y[3] = {0,0,0.05};
                    double ori[3] = {90,0,-135};
                    reposicionar_time(x,y,ori,replacerClient);
                }
                if(refereeClient->getLastFoulColor() == VSSRef::Color::BLUE){
                    double x[3] = {0.7,0.3,0.5};
                    double y[3] = {0,0,0};
                    double ori[3] = {90,0,0};
                    reposicionar_time(x,y,ori,replacerClient);
                }
           }

           if(refereeClient->getLastFoul() == VSSRef::Foul::GOAL_KICK){
               //posição da bola = 0.6,0.35
               //posição goleiro = 0.65, 0.3
               if(refereeClient->getLastFoulColor() == VSSRef::Color::BLUE){
                   double x[3] = {0.7,-0.2,-0.2};
                   double y[3] = {0,-0.4,0.4};
                   double ori[3] = {90,0,0};
                   reposicionar_time(x,y,ori,replacerClient);
               }
               if(refereeClient->getLastFoulColor() == VSSRef::Color::YELLOW){
                   double x[3] = {0.7,0.67,0.67};
                   double y[3] = {0,-0.37,0.37};
                   double ori[3] = {90,165,195};
                   reposicionar_time(x,y,ori,replacerClient);

               }
           }

           if(refereeClient->getLastFoul() == VSSRef::Foul::PENALTY_KICK){
               double gol[2] = {25,-25};
               int sorteio = rand()%2 ;
               double orientacao = gol[sorteio];
               cout<<sorteio<<"  "<<orientacao<<endl;
               if(refereeClient->getLastFoulColor() == VSSRef::Color::BLUE){
                   double x[3] = {0.7,-0.05,-0.05};
                   double y[3] = {0,-0.3,0.3};
                   double ori[3] = {90,0,0};
                   reposicionar_time(x,y,ori,replacerClient);
               }
               if(refereeClient->getLastFoulColor() == VSSRef::Color::YELLOW){
                   double x[3] = {0.7,0.05,-0.3};
                   double y[3] = {0,0,0};
                   double ori[3] = {90,0,orientacao};
                   reposicionar_time(x,y,ori,replacerClient);
               }
           }
        }
        //ENVIA REPOSICIONAMENTO AZUL
        else{ //AZUL
             if(refereeClient->getLastFoul() == VSSRef::Foul::FREE_BALL){
                   double x[3] = {-0.7,-0.3,-0.3};
                   double y[3] = {0,0,0.3};
                   double ori[3] = {90,0,0};

                   if(refereeClient->getLastFoulQuadrant() == VSSRef::Quadrant::QUADRANT_1){
                       x[2] = 0.18;
                       y[2] = 0.4;
                   }
                   if(refereeClient->getLastFoulQuadrant() == VSSRef::Quadrant::QUADRANT_4){
                       x[2] = 0.18;
                       y[2] = -0.4;
                   }
                   if(refereeClient->getLastFoulQuadrant() == VSSRef::Quadrant::QUADRANT_2){
                       x[2] = -0.59;
                       y[2] = 0.4;
                       ori[2] = -5;
                   }
                   if(refereeClient->getLastFoulQuadrant() == VSSRef::Quadrant::QUADRANT_3){
                       x[2] = -0.59;
                       y[2] = -0.4;
                       ori[2] = 5;
                   }
                   reposicionar_time(x,y,ori,replacerClient);
           }

           if(refereeClient->getLastFoul() == VSSRef::Foul::KICKOFF){
               if(refereeClient->getLastFoulColor() == VSSRef::Color::BLUE){
                   double x[3] = {-0.7,-0.3,-0.05};
                   double y[3] = {0,0,0.05};
                   double ori[3] = {90,0,-45};
                   reposicionar_time(x,y,ori,replacerClient);
               }
               if(refereeClient->getLastFoulColor() == VSSRef::Color::YELLOW){
                   double x[3] = {-0.7,-0.3,-0.5};
                   double y[3] = {0,0,0};
                   double ori[3] = {90,0,0};
                   reposicionar_time(x,y,ori,replacerClient);
               }
           }

           if(refereeClient->getLastFoul() == VSSRef::Foul::GOAL_KICK){
               //posição da bola = 0.6,0.35
               //posição goleiro = 0.65, 0.3

               if(refereeClient->getLastFoulColor() == VSSRef::Color::BLUE){
                   double x[3] = {-0.7,-0.67,-0.67};
                   double y[3] = {0,-0.37,0.37};
                   double ori[3] = {90,195,165};
                   reposicionar_time(x,y,ori,replacerClient);

               }
               if(refereeClient->getLastFoulColor() == VSSRef::Color::YELLOW){
                   double x[3] = {-0.7,0.2,0.2};
                   double y[3] = {0,-0.4,0.4};
                   double ori[3] = {90,0,0};
                   reposicionar_time(x,y,ori,replacerClient);
               }
           }

           if(refereeClient->getLastFoul() == VSSRef::Foul::PENALTY_KICK){
               double gol[4] = {25,15,-15,-25};
               int sorteio = rand()%4 ;
               double orientacao = gol[sorteio];
               cout<<sorteio<<"  "<<orientacao<<endl;
               if(refereeClient->getLastFoulColor() == VSSRef::Color::BLUE){
                   double x[3] = {-0.7,-0.05,0.3};
                   double y[3] = {0,0,0};
                   double ori[3] = {90,0,orientacao};
                   reposicionar_time(x,y,ori,replacerClient);
               }
               if(refereeClient->getLastFoulColor() == VSSRef::Color::YELLOW){
                   double x[3] = {-0.7,0.05,0.05};
                   double y[3] = {0,-0.3,0.3};
                   double ori[3] = {90,0,0};
                   reposicionar_time(x,y,ori,replacerClient);
               }
           }
       }











        // Debugging vision
        fira_message::sim_to_ref::Environment lastEnv = visionClient->getLastEnvironment();
        if(lastEnv.has_frame()) {

            // Taking last frame
            fira_message::Frame lastFrame = lastEnv.frame();
            //Ball info:
            fira_message::Ball ball = lastFrame.ball();
            // Field
            const fira_message::Field & field = lastEnv.field();


            fira_message::Robot b0 = lastFrame.robots_blue(0);
            fira_message::Robot b1 = lastFrame.robots_blue(1);
            fira_message::Robot b2 = lastFrame.robots_blue(2);
            //Yellow
            fira_message::Robot y0 = lastFrame.robots_yellow(0);
            fira_message::Robot y1 = lastFrame.robots_yellow(1);
            fira_message::Robot y2 = lastFrame.robots_yellow(2);

            estrategia.predict_ball(ball);

            estrategia.atualiza_pos(b0,b1,b2,y0,y1,y2);


            refereeClient->getLastFoul() == VSSRef::Foul::GAME_ON;  // Status de jogo

            if(!ourSideIsLeft){
                estrategia.strategy_yellow(y0,y1,y2,b0,b1,b2,ball,field,"ultimo_comando");
            }else{
                estrategia.strategy_blue(b0,b1,b2,y0,y1,y2,ball,field,"ultimo_comando");
            }
            //Enviando velocidades para os robos
            for(int i = 0;i < estrategia.qtdRobos;i++)
                actuatorClient->sendCommand(i,estrategia.vRL[i][1],estrategia.vRL[i][0]);
                //actuatorClient->sendCommand(i, estrategia_amarela.vRL[i][1], estrategia_amarela.vRL[i][0]);









            if(0){
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
            }
        }
        /*
        // Sending robot commands for robot 0, 1 and 2
        actuatorClient->sendCommand(0, 0, 0);
        actuatorClient->sendCommand(1, 0, 0);
        actuatorClient->sendCommand(2, 0, 0);

        // If is kickoff, send this test frame!
        if(refereeClient->getLastFoul() == VSSRef::Foul::PENALTY_KICK) {

            replacerClient->placeRobot(0, ourSideIsLeft ? -0.2 : 0.2, 0, 0);
            replacerClient->placeRobot(1, ourSideIsLeft ? -0.2 : 0.2, 0.2, 0);
            replacerClient->placeRobot(2, ourSideIsLeft ? -0.2 : 0.2, -0.2, 0);
            replacerClient->sendFrame();
        }
        */
        std::cout << "Condição de" << getFoulNameById(refereeClient->getLastFoul()).toStdString() ;
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

    return a.exec();
}
