Sequ�ncia de desenvolvimento:

1. aceler�metro_1

A fun��o deste c�digo � escrever e ler em um dos registradores do aceler�metro para valida��o da interface SPI
de comunica��o entre MSP430 e o aceler�metro.

2. aceler�metro_2

Este c�digo possibilita a verifica��o do comportamento do carro atrav�s de LEDs. Dependendo da inclina��o do
aceler�metro (para frente e/ou esquerda ou direita) os LEDs que indicam os movimentos ser�o acesos ou apagados.
Quando aceso, o LED vermelho indica que o carro se desloca para frente. O acendimento de qualquer LED verde ao
mesmo tempo que o vermelho indica uma trajet�ria circular no sentido hor�rio ou anti-hor�rio. O apagamento do
LED vermelho em qualquer situa��o indica repouso do carro.

3. aceler�metro_3

Este c�digo � semelhante ao aceler�metro_1. Nesse caso, a comunica��o SPI � realizada a 3 fios e as fun��es de
leitura e escrita foram modificadas.

4. aceler�metro_4

Este c�digo � semelhante ao aceler�metro_2. Nesse caso, a comunica��o SPI � realizada a 3 fios e as fun��es de
leitura e escrita foram modificadas.

5. rf_1

Este c�digo faz parte de um c�digo ainda mais complexo desenvolvido pela Texas Instruments. O c�digo original,
que n�o encontra-se no reposit�rio, foi escrito para atender ao uso de diferentes plataformas do fabricante.
Por isso, houve a necessidade de editar o c�digo para diminuir o n�mero de arquivos, condensando-os em apenas
3 arquivos. Ap�s a edi��o, esse c�digo consta de 1 arquivo .h contendo declara��es, 1 arquivo .c contendo
fun��es, e 1 arquivo .c contendo a fun��o principal e a rotina de interrup��o.

6. rf_2

Este c�digo � uma altera��o do rf_1. As altera��es foram realizadas de modo a deixar o c�digo mais leg�vel.
Foram adotados os mesmos padr�es de escrita dos c�digos de valida��o do aceler�metro. O algoritmo do c�digo
desenvolvido pela Texas Instruments foi mantido.

7. integra��o_1

Este c�digo foi adaptado de modo a manter em funcionamento simult�neo o aceler�metro e o RF. Desse modo, os
leds LED1 e LED2 acendem se a comunica��o MSP430 - ACELER�METRO e MSP430 - RF forem estabelecidas,
respectivamente. Ap�s o acendimento desses leds � enviado um pacote por RF cuja fun��o � acender um LED verde
em outra placa onde deve estar gravado o c�digo rf_2.