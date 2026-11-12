// A starter robot for rules v2, on the Clunkyard Java binding (package
// clunkyard, which the arena provides). It sweeps a medium scan to find a
// robot, or points it where a passive contact was heard, narrows the scan
// to hold it, leads it with shots whose power suits the range, fires only
// when the turret is on target, keeps battery back for driving and keeps
// out of the storm.
//
// The trade-offs (gun(), scanner() and battery() have the numbers):
//   scan     a narrow arc reaches further; a wide one finds robots nearby
//   power    heavy shots hit harder for less battery, but fly slowly
//   battery  shots and driving share it, and it refills slowly
//   turret   it turns at a limited rate; gun().aimNow() is where it points now
//   cues     passive contacts (a ping, a scan beam, a shot heard) say which
//            way to look, not where to shoot: they have no id and a noisy
//            bearing, so sensors().nearest() leaves them out
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range)
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import clunkyard.*;

public final class Bot extends Robot {
  /** Enough to turn against the brake in the force rulesets. */
  private static final double SEARCH_TURN = 0.33;

  private boolean free; // flying free, robots may be above or below

  /** A turn or pitch-turn control towards an angle, full at 45 degrees off. */
  private double steer(double towards, double current) {
    return angles().clamp(angles().angleDiff(towards, current) / 45);
  }

  @Override
  protected void onStart() {
    free = match().is3d() && arena().gravity().z() == 0;
  }

  @Override
  protected void onTurn() {
    Me me = me();
    Vec3 position = me.position();
    drive().forces(Drive.brake(0.2));
    var nearest = sensors().nearest();
    var heard = tracker().cue(); // the freshest passive contact, for a few seconds
    if (nearest.isPresent()) {
      Contact target = nearest.get();
      double throttle = target.range() > 350 ? 0.7 : target.range() < 200 ? -0.4 : 0.2;
      drive().set(throttle, steer(target.bearing(), me.heading()), steer(target.elevation(), me.pitch()));
      scanner().set(target.bearing(), 20, target.elevation()); // narrow: holds it, reaching about 735
      // The heaviest shot that still covers the range in about a second,
      // aimed where the target will be, lobbed under gravity.
      double power = gun().powerForFlightTime(target.range(), 1);
      var aim = gun().lead(target, power);
      double heading = aim.map(Solution::heading).orElse(target.bearing());
      double pitch = aim.map(Solution::pitch).orElse(target.elevation());
      gun().aim(heading, pitch);
      Direction now = gun().aimNow();
      boolean onTarget = Math.abs(angles().angleDiff(heading, now.heading())) < 3 && Math.abs(pitch - now.pitch()) < 3;
      if (onTarget && gun().canFire(power, 10)) gun().fire(power);
      status().set("TRACK", target.range());
    } else if (heard.isPresent()) {
      // Head for where it was heard, sweeping an arc that still reaches
      // it across the bearing's error (and, flying free, the elevation's).
      Cue cue = heard.get();
      double bearing = angles().bearingTo(position, cue.position()), distance = angles().distance(position, cue.position());
      double elevation = free ? angles().elevationTo(position, cue.position()) : 0;
      double error = Math.min(cue.bearingError(), 30);
      int tick = clock().tick();
      double across = (tick % 5 - 2) / 2.0 * error;
      double up = free ? ((tick / 5) % 3 - 1) * error / 2 : 0;
      drive().set(0.7, steer(bearing, me.heading()), steer(elevation, me.pitch()));
      scanner().set(bearing + across, scanner().arcForReach(distance + 100), elevation + up);
      status().set("CUE", distance);
    } else {
      drive().set(0.5, SEARCH_TURN, steer(0, me.pitch()));
      // Sweeps the circle in half a second, flying free at three heights in turn.
      double pitch = free ? ((clock().tick() / 10) % 3 - 1) * 35 : 0;
      scanner().set(angles().normalize(scanner().last().heading() + 36), 40, pitch);
      status().set("SEARCH");
    }
    // Low battery: drive gently so the shots can still be paid for.
    if (battery().charge() < battery().capacity() / 4)
      drive().set(Drive.throttle(drive().get("throttle").orElse(0) * 0.5));
    // The storm hurts inside: near its edge, drive straight away from its centre.
    nav().storm().ifPresent(storm -> {
      if (storm.margin(position).orElse(-1e9) > -60) {
        double away = angles().bearingTo(storm.center(), position);
        drive().set(Drive.throttle(0.8).turn(angles().clamp(angles().angleDiff(away, me.heading()) / 30)));
      }
    });
    // Sudden death: head back inside the safe zone.
    nav().zone().ifPresent(zone -> {
      if (zone.margin(position).orElse(1e9) < 60) {
        double climb = zone.shape() == Shape.SPHERE ? angles().elevationTo(position, zone.center()) : 0;
        drive().set(0.8, angles().clamp(angles().angleDiff(angles().bearingTo(position, zone.center()), me.heading()) / 30),
            angles().clamp((climb - me.pitch()) / 30));
      }
    });
  }

  public static void main(String[] args) {
    Clunkyard.run(new Bot());
  }

}
