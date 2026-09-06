# AgriLink Vehicle State Model



## Purpose



This document describes the state model used by the AgriLink embedded vehicle-control ECU prototype.



The implementation contains three related but distinct state concepts:



1. `VehicleState` - vehicle operational/safety target state.

2. `SafetyStatus` - result produced by the safety supervisor.

3. `SystemState` - simplified system state exposed through the heartbeat subsystem.



They are intentionally documented separately rather than being treated as one state machine.



---



## 1. Vehicle State Model



The `VehicleState` enumeration defines the following states:



\- `INIT`

\- `READY`

\- `AUTONOMOUS`

\- `DEGRADED`

\- `FAULT`

\- `SAFE_STOP`

\- `EMERGENCY_STOP`

\- `RECOVERY`



The current safety supervisor directly produces the following vehicle target states:



| Safety condition | Safety status | Vehicle target state | Control |

|---|---|---|---|

| All monitored conditions healthy | `NORMAL` | `READY` | Allowed |

| Sensor invalid | `DEGRADED_SENSORS` | `DEGRADED` | Inhibited |

| Communication unhealthy | `DEGRADED_COMM` | `DEGRADED` | Inhibited |

| Sensor + communication unhealthy | `SAFE_STOP` | `SAFE_STOP` | Inhibited |

| Vehicle speed above 10 m/s | `EMERGENCY_STOP` | `EMERGENCY_STOP` | Inhibited |



### State-oriented view



```text

                         +----------------+

                         |      INIT      |

                         +-------+--------+

                                 |

                                 | subsystem initialization

                                 v

                         +----------------+

                         |     READY      |

                         +-------+--------+

                                 |

                                 | safety permits control

                                 v

                         +----------------+

                         | CONTROL ENABLED|

                         +-------+--------+

                                 |

              +------------------+------------------+

              |                  |                  |

              | sensor fault     | communication   | overspeed

              |                  | fault            |

              v                  v                  v

       +-------------+    +-------------+    +------------------+

       |  DEGRADED   |    |  DEGRADED   |    | EMERGENCY_STOP   |

       +------+------+    +------+------+    +------------------+

              \\                  /

               \\                /

                \\ combined fault

                 v

              +-------------+

              |  SAFE_STOP  |

              +-------------+
