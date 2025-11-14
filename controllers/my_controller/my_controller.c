/*
  Exemplo introdutório de uso do WeBots
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>   // para sqrt

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

/*
 * You may want to add macros here.
 */

//TIME_STEP é o incremento de tempo usado na simulação
//512 é um valor MUITO alto.... mas ajuda para ler o que está sendo mostrado no console
//para a simulação final, melhor usar valores menores.... 16 ou 32!
#define TIME_STEP 32

#define QtddSensoresProx 8
#define QtddLeds 10
#define TamanhoTexto 256
#define QtddCaixa 20

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

int main(int argc, char **argv) {

  int i=0;
  char texto[TamanhoTexto]={0};
  double LeituraSensorProx[QtddSensoresProx];
  double posAntes[QtddCaixa][3];
  double posDepois[QtddCaixa][3];
  int achei = 0;
  double tempoInicio = 0;

  /* necessary to initialize webots stuff */
  wb_robot_init();

  //configurando MOTORES
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito  = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito , INFINITY);

  //motores parados
  wb_motor_set_velocity(MotorEsquerdo,0);
  wb_motor_set_velocity(MotorDireito,0);

  //configura Sensores de Proximidade
  WbDeviceTag SensorProx[QtddSensoresProx];
  char nomeSensor[10]={0};

  for(i=0;i<QtddSensoresProx;i++){
     sprintf(nomeSensor,"ps%d",i); //form os nomes dos sensores
     SensorProx[i] = wb_robot_get_device(nomeSensor);
     wb_distance_sensor_enable(SensorProx[i],TIME_STEP);
  }

  //config leds
  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  wb_led_set(Leds[0],-1);

  // ----------- Caixas -----------
  WbNodeRef caixa[QtddCaixa];
  char nomeCaixa[10]={0};

  for(i=0;i<QtddCaixa;i++){
      sprintf(nomeCaixa,"CAIXA%02d",i);
      caixa[i] = wb_supervisor_node_get_from_def(nomeCaixa);
      if(caixa[i]!=NULL)
         printf("%2d. %s  -  %p\n",i,nomeCaixa,(void*)caixa[i]);
      else
         printf("Falha ao carregar a posição da %s\n",nomeCaixa);
  }
  printf("\n\n CAIXAS OK  \n\n");

  for(i=0;i<QtddCaixa;i++){
    if(caixa[i]!=NULL){
      const double *p = wb_supervisor_node_get_position(caixa[i]);
      posAntes[i][0]=p[0];
      posAntes[i][1]=p[1];
      posAntes[i][2]=p[2];
    }
  }

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */

  while (wb_robot_step(TIME_STEP) != -1) {

    // ----------- Limpa texto para próxima iteração -----------
    memset(texto,0,TamanhoTexto);

    // ----------- Lendo sensores de proximidade -----------
    for(i=0;i<QtddSensoresProx;i++){
       LeituraSensorProx[i]= wb_distance_sensor_get_value(SensorProx[i])-60;

       char temp[64];
       snprintf(temp, sizeof(temp), "|%d: %6.0f  ", i, LeituraSensorProx[i]);
       strncat(texto, temp, TamanhoTexto - strlen(texto) - 1);
    }

    // ----------- Lendo posição das caixas -----------
    strncat(texto, "\n           X       Y      Z\n",
            TamanhoTexto - strlen(texto) - 1);

    for(i=0;i<QtddCaixa;i++){
      if(caixa[i]!=NULL){
        const double *PosicaoCaixa = wb_supervisor_node_get_position(caixa[i]);
        char temp[128];
        snprintf(temp, sizeof(temp),
                 "CAIXA%02d %5.2f   %5.2f  %5.2f\n",
                 i, PosicaoCaixa[0], PosicaoCaixa[1], PosicaoCaixa[2]);
        strncat(texto, temp, TamanhoTexto - strlen(texto) - 1);
      }
    }

    // ----------- Mostra no console -----------
    printf("%s\n",texto);

    // ----------- Pisca o LED -----------
    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1); 

    if(achei==0) {
  
      // Se o sensor frontal detectar algo, tenta empurrar
      double frente = wb_distance_sensor_get_value(SensorProx[0]) +
                      wb_distance_sensor_get_value(SensorProx[7]);
  
      if(frente > 450.0) { // valor alto = algo na frente
        printf("CAIXA detectada! Empurrando...\n");
        wb_motor_set_velocity(MotorEsquerdo, 3.0);
        wb_motor_set_velocity(MotorDireito , 3.0);
        tempoInicio = wb_robot_get_time();
  
        // empurra por 1 segundo
        while(wb_robot_get_time() - tempoInicio < 1.0) {
          if (wb_robot_step(TIME_STEP) == -1)
            break;
        }
  
        // Lê de novo as posições das caixas
        for(i=0;i<QtddCaixa;i++){
          if(caixa[i]!=NULL){
            const double *p = wb_supervisor_node_get_position(caixa[i]);
            posDepois[i][0]=p[0];
            posDepois[i][1]=p[1];
            posDepois[i][2]=p[2];
          }
        }
  
        // Verifica se alguma caixa se moveu
        for(i=0;i<QtddCaixa;i++){
          double dx = posDepois[i][0]-posAntes[i][0];
          double dy = posDepois[i][1]-posAntes[i][1];
          double dist = sqrt(dx*dx + dy*dy);
          if(dist > 0.02){ // se moveu 2 cm, achou a leve!
            achei = 1;
            printf("ACHEI A CAIXA LEVE! (%d)\n", i);
            break;
          }
        }
  
        // Atualiza posAntes para continuar testando outras
        for(i=0;i<QtddCaixa;i++){
          posAntes[i][0]=posDepois[i][0];
          posAntes[i][1]=posDepois[i][1];
          posAntes[i][2]=posDepois[i][2];
        }

        // Se AINDA não achou a caixa leve, gira para procurar outra
        if (!achei) {
          printf("Não era a caixa leve, girando para procurar outra...\n");
          tempoInicio = wb_robot_get_time();
          wb_motor_set_velocity(MotorEsquerdo, 2.5);
          wb_motor_set_velocity(MotorDireito ,-2.5);

          while (wb_robot_get_time() - tempoInicio < 0.8) { // gira por 0.8 s
            if (wb_robot_step(TIME_STEP) == -1)
              break;
          }
        }

      }
      else {
        // anda e gira levemente procurando
        wb_motor_set_velocity(MotorEsquerdo, 3.0);
        wb_motor_set_velocity(MotorDireito , 2.75);
      }
    }
    else {
      // Se já achou a caixa leve, gira no próprio eixo
      wb_motor_set_velocity(MotorEsquerdo, 2.5);
      wb_motor_set_velocity(MotorDireito ,-2.5);
    }

  };

  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */

  wb_robot_cleanup();

  return 0;
}

