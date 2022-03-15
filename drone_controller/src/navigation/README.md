# Códigos e documentação da Navegação
Namespace: `navigation`.

## Pipeline de planejamento
Para executar a pipeline de planejamento, use o comando
```
roslaunch drone_controller planning.launch
```

### Planning Manager
Faz o gerenciamento do planejamento. Se inscreve nos tópicos do mapa, odometria do drone e posição desejada e chama o serviço que implementa o planejador escolhido para obter a trajetória.

### Planejadores Disponíveis
Para escolher o planejador que será utilizado, use o parâmetro `planning_manager/planner`. As opções disponíveis até então são: `grid_planner`.

Os planejadores são implementados como serviços e, para manter a generalidade, todos usam a mesma mensagem (`custom_msgs_srvs/srv/Plan.srv`). Os parâmetros específicos de cada abordagem de planejamento podem ser configuradas através do `rosparam`.

#### Grid Planner
Parâmetros: 
- `name: cell_size; default: 2`. Determina o tamanho da aresta da célula usada na discretização do mapa.