# PIC-JR 2019: Projeto de desenvolvimento de um kit de robótica de baixo custo para auxílio em práticas pedagógicas

Considerando os constantes avanços tecnológicos pelo qual temos passado atualmente e a consequente inserção de novas tecnologias em nosso dia-a-dia, acabamos por absorver estas inovações mas sem o domínio completo do seu funcionamento. Disso decorre a aquisição de conhecimento superficial que se perde juntamente com a obsolescência da tecnologia quando substituída por outra. Há de se modificar essa tônica para que nossos alunos possam adquirir conhecimento mais aprofundado dos fenômenos físicos por trás das inovações tecnológicas para utilizá-las com senso crítico e colaborar no futuro para a criação de novas tecnologias. Nesse sentido, propomos neste projeto o desenvolvimento de kits educacionais de robótica de baixo custo como forma de auxiliar as práticas pedagógicas no ensino de Física no Ensino Médio.

## Objetivo Geral:
Este projeto objetiva o desenvolvimento de um kit de robótica de baixo custo que propicie a compreensão, por parte dos alunos do Ensino Médio de escolas públicas do Espírito Santo, a respeito dos fundamentos teóricos e práticos ensinados nas disciplinas de Física dos três anos do ensino médio relativos aos conceitos, às aplicações, à modelagem, ao controle e à programação de robôs móveis. 

## Objetivos Específicos:
- Desenvolver modelo 3D em CAD para a plataforma robótica, que será usado para fabricação das peças na impressora 3D e também será exportado para outro software de simulação robótica 3D.
- Desenvolver e testar os algoritmos que farão parte da biblioteca de software para controle da plataforma robótica tanto em ambiente simulado 3D quanto ambiente físico real.
- Elaborar roteiro com o passo-a-passo para montagem, configuração da plataforma robótica e calibração dos sensores e atuadores.
- Elaborar roteiros de aulas práticas envolvendo os sensores e atuadores utilizados na plataforma robótica, incluindo também missões a serem desempenhadas pela plataforma robótica como um todo. 
- Desenvolver o trabalho em equipe voltado a resolução de problemas e disseminação de conhecimento, além de desenvolver o pensamento computacional aplicado a resolução de problemas.

## Componentes Físicos do Kit de Robótica:
Os kits de robótica serão compostos de sensores e atuadores conforme listado abaixo:
-	**Placa Ponte H**: este componente eletrônico possui um circuito Ponte H para controlar cargas indutivas como relés, solenoides, motores DC e motores de passo. Com este circuito Ponte H é possível controlar independentemente a velocidade e rotação de 2 motores DC que farão girar as rodas do robô diferencial. Os conceitos envolvidos na construção e funcionamento deste circuito normalmente são vistos no conteúdo de Eletricidade na Física do 3º ano. 
-	**Motor DC**: este atuador é composto de um motor elétrico que faz girar a roda do robô, cuja velocidade de giro em rotações por minuto (RPM) pode ser controlada variando a tensão de alimentação. Os conceitos aqui envolvidos normalmente são vistos no conteúdo de Eletromagnetismo na Física do 3º ano.
-	**Roda pequena**: este componente mecânico é responsável por deslocar o robô em superfícies planas levando em consideração o torque do motor e as forças de atrito com o solo. Os conceitos relacionados as forças e atritos envolvidos no deslocamento de corpos é normalmente visto em cinemática/dinâmica na Física do 1º ano.
-	**Sensores de distância ultrassônico**: este sensor mede a distância até os obstáculos por meio de ondas sonoras, sendo muito importante para evitar colisões do robô com os obstáculos. A teoria envolvida no funcionamento deste sensor é normalmente vista no conteúdo sobre Ondas em Física no 2º ano.
-	**Acelerômetro e giroscópio**: estes sensores medem a aceleração e velocidade nos três eixos (x,y,z) com 6 graus de liberdade e serão importantes para controlar a velocidade e direção do robô diferencial. Os conceitos envolvidos na sua construção e funcionamento são amplamente aplicados em conteúdos de Física do 1º ano, como movimento em plano inclinado, movimento uniformemente variado, movimento pendular, movimento uniforme.
-	**Sensor de velocidade ótico**: este sensor mede a velocidade das rodas do robô por meio de um dispositivo ótico para que seja possível implementar mecanismos de controle além de determinar a velocidade do robô.
-	**Câmeras RGB**: o microcontrolador ESP32 CAM possui uma câmera embutida que será útil para desempenhar missões mais complexas que exijam a interação com o ambiente, além é claro de ser muito importante para explicar fenômenos de distorção de imagens em lentes, conceito este abordado em Ótica na Física do 2º ano.
-	**Placa ESP32 CAM**: este microcontrolador será instalado no robô para realizar todo o processamento embarcado, controlar os motores acoplados as rodas, ler diversas medidas de sensores como da câmera embutida nele e comunicar com outros dispositivos via rede Wireless.

Além desses equipamentos e componentes, utilizamos a impressoras 3D GTMax3D (modelo Pro Core H4) para impressão de peças, tais como:
- [Suporte para câmera RGB](./doc/Suporte%20CAM.stl); 
-	[Suporte para sensor ultrassônico](./doc/Base%20Sensor.stl);
-	[Chassi para robô com tração nas duas rodas](./doc/base.stl);
-	Roda boba para aplicações em robótica móvel.
-	[Roda para aplicações em robótica móvel](./doc/Roda.stl);
