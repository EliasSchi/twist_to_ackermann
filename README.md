
## Summary 
 Translates twist commands to ackermann drive commands, to be used in car-like robots.

## Topics

### Publishes
- `/ack_vel`: A stamped  ackermann drive command resulting from the translation.

### Subscribes
- `/cmd_vel`: The unstamped twist message to convert.
