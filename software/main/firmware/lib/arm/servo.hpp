namespace arm {


class Servo {
public:
  Servo(types::u8 servo_pin, types::u8 servo_dir, types::u8 gear_ratio = 4, types::u16 min_pulse = 500, types::u16 max_pulse = 2500, types::u16 dead_band = 7, types::u16 max_speed = 375);
  void begin();
  void move(types::u16 target_ang); //target ang * 10: 0 - 7200
private:
  RP2040_PWM* PWM_Instance;

  float _frequency;
  float _duty_cycle;

  types::u8 _servo_pin;
  types::u8 _servo_dir;

  types::u8 _gear_ratio;

  //in microseconds
  types::u16 _dead_band;
  types::u16 _period = 20000;
  types::u16 _min_pulse;
  types::u16 _max_pulse;

  // ang * 10: 0 - 7200
  types::u16 _curr_ang = 0;

  // degs per second
  types::u16 _max_speed;
};

}
