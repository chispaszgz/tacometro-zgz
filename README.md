# Tacómetro ZgZ

Banco de rodillos para verificar que patinetes y vehículos de movilidad
personal no superan la velocidad máxima permitida.

El aparato mide sobre rodillo, muestra la prueba en vivo en un OLED, la guarda
en su memoria interna e imprime un ticket con la gráfica y un código QR.

## Consola web

`index.html` es una aplicación que se conecta al tacómetro **por USB** usando
la API Web Serial, sin instalar nada. Permite:

- consultar identidad, versión de firmware y estado del almacenamiento
- editar el identificador del aparato, el agente y la nota
- listar las pruebas guardadas, verlas con su curva y descargarlas en CSV
- grabar una versión nueva de firmware

Requiere **Chrome o Edge de escritorio** y que la página se sirva por HTTPS o
desde `localhost`: Web Serial exige contexto seguro y no funciona abriendo el
archivo con doble clic.

## Firmware

`sketch/` contiene el programa del ESP32-C6 (placa MakerGO ESP32 C6 SuperMini,
core esp32 3.3.10, con *USB CDC On Boot* activado).

`firmware/` contiene los binarios compilados y el manifiesto que usa la consola
web para grabarlos. Se graban el gestor de arranque, la tabla de particiones y
la aplicación; **la partición de datos no se toca, así que las pruebas
almacenadas se conservan** al actualizar.

## Protocolo por USB

Órdenes en texto, una por línea, con respuestas en JSON de una sola línea:

| Orden | Devuelve |
|---|---|
| `PING` | comprobación de vida |
| `INFO` | identidad, versión y estado |
| `LIST` | pruebas guardadas |
| `GET <nombre>` | contenido del CSV en base64 |
| `DEL <nombre>` | borra una prueba |
| `CFG` | agente, nota y tipo de impresora |
| `SETID` / `SETAG` / `SETNOTA` | escritura de esos campos |
| `SETIMP cable|bluetooth` | elige la impresora |
| `SETMANDO <mac>` | dirección BLE del mando auxiliar |

Las trazas de depuración del firmware salen por el mismo puerto, así que
cualquier línea que no empiece por `{` debe ignorarse.
