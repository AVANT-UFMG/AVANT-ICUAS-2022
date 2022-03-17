# Códigos e documentação da Integração de Sistemas
## Mapeamento
Para iniciar o mapeamento, execute o comando
```
roslaunch drone_controller mapping.launch
```

Usamos o pacote [octomap_mapping](http://wiki.ros.org/octomap_mapping) da biblioteca [OctoMap](https://octomap.github.io/) para gerar o mapa dinamicamente. Assim, é gerado um OctoMap (3D) e o Occupancy Grid (2D) pode ser lido no tópico `/projected_map` publicado pelo próprio `octomap_mapping`, que é uma projeção do OctoMap no chão.
