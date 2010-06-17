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

O objetivo deste c�digo � apenas alterar o modo de funcionamento da interface SPI. Nos c�digos "aceler�metro_1"
e "aceler�metro_2" a comunica��o atrav�s da interface SPI � contemplada a 4 fios. Este c�digo altera a
configura��o dessa interface para que funcione a 3 fios. Isso tornou-se necess�rio devido ao fato de que a
interface SPI dever� ser compartilhada entre o aceler�metro e o controlador de RF, cuja comunica��o SPI
disponibilizada pelo fabricante � realizada a 3 fios. Nesta situa��o, na comunica��o SPI, o aceler�metro e o
controlador de RF ser�o os dispositivos escravos (slaves) e o MSP430 ser� o dispositivo mestre (master). Al�m
de alterar a configura��o da interface SPI, o c�digo ser� modularizado, ou seja, o c�digo ser� dividido em 3
arquivos, resultando em 1 arquivo .h contendo declara��es, 1 arquivo .c contendo fun��es e 1 arquivo .c
contendo a fun��o principal. 

4. rf_1

Este c�digo faz parte de um c�digo ainda mais complexo desenvolvido pela Texas Instruments. O c�digo original,
que n�o encontra-se no reposit�rio, foi escrito para atender ao uso de diferentes plataformas do fabricante.
Por isso, houve a necessidade de editar o c�digo para diminuir o n�mero de arquivos, condensando-os em apenas
3 arquivos. Ap�s a edi��o, esse c�digo consta de 1 arquivo .h contendo declara��es, 1 arquivo .c contendo
fun��es, e 1 arquivo .c contendo a fun��o principal e a rotina de interrup��o.
