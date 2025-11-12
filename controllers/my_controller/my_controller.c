/*
  Exemplo introdutório de uso do WeBots
 */


/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */

#include <stdio.h>
#include <string.h>

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
#define TIME_STEP 512

#define QtddSensoresProx 8
#define QtddLeds 10
#define TamanhoTexto 256
#define QtddCaixa 8



/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

int main(int argc, char **argv) {

  int i=0;
  char texto[TamanhoTexto]={0};
  double LeituraSensorProx[QtddSensoresProx];
  //double AceleradorDireito=1.0, AceleradorEsquerdo=1.0;
  double posAntes[QtddCaixa][3];
  double posDepois[QtddCaixa][3];
  int achei = 0;
  double tempoInicio = 0;


  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

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

  // ----------- Caixas (parte do segundo código) -----------
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

    // ----------- Lendo sensores de proximidade -----------
    for(i=0;i<QtddSensoresProx;i++){
       LeituraSensorProx[i]= wb_distance_sensor_get_value(SensorProx[i])-60;
       sprintf(texto,"%s|%d: %6.0f  ",texto,i,LeituraSensorProx[i]);
    }

    // ----------- Lendo posição das caixas -----------
    strcat(texto, "\n           X       Y      Z\n");
    for(i=0;i<QtddCaixa;i++){
      if(caixa[i]!=NULL){
        const double *PosicaoCaixa = wb_supervisor_node_get_position(caixa[i]);
        char temp[128];
        sprintf(temp,"CAIXA%02d %5.2f   %5.2f  %5.2f\n",i,PosicaoCaixa[0],PosicaoCaixa[1],PosicaoCaixa[2]);
        strcat(texto,temp);
      }
    }

    // ----------- Mostra no console -----------
    printf("%s\n",texto);

    // ----------- Pisca o LED -----------
    wb_led_set(Leds[0], wb_led_get(Leds[0])*-1); 

    // Movimento baseado na primeira caixa (só como exemplo)
    const double *PosicaoCaixa = NULL;
    for(i=0;i<QtddCaixa;i++){
      if(caixa[i]!=NULL){
        PosicaoCaixa = wb_supervisor_node_get_position(caixa[i]);
        break; // usa a primeira caixa válida
      }
    }

    if(achei==0) {
  
    // Se o sensor frontal detectar algo, tenta empurrar
    double frente = wb_distance_sensor_get_value(SensorProx[0]) + wb_distance_sensor_get_value(SensorProx[7]);
  
    if(frente > 450.0) { // valor alto = algo na frente
      printf("CAIXA detectada! Empurrando...\n");
      wb_motor_set_velocity(MotorEsquerdo, 3.0);
      wb_motor_set_velocity(MotorDireito , 3.0);
      tempoInicio = wb_robot_get_time();
  
      // empurra por 1 segundo
      while(wb_robot_get_time() - tempoInicio < 1.0) {
        wb_robot_step(TIME_STEP);
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
      // Se ele não achar a caixa certa ele gira no próprio eixo para mudar a direção
      wb_motor_set_velocity(MotorEsquerdo, 2.5);
      wb_motor_set_velocity(MotorDireito ,-2.5);
    }
    else {
      // anda e gira levemente procurando
      wb_motor_set_velocity(MotorEsquerdo, 3.0);
      wb_motor_set_velocity(MotorDireito , 2.95);
    }
  }
  else {
    // Se já achou a caixa leve, gira no próprio eixo
    wb_motor_set_velocity(MotorEsquerdo, 2.5);
    wb_motor_set_velocity(MotorDireito ,-2.5);
  }

    // ----------- Limpa texto para próxima iteração -----------
    memset(texto,0,TamanhoTexto);
  };

  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */

  wb_robot_cleanup();

  return 0;
}
