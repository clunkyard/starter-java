// A starter robot for rules v2, on the Clunkyard Java binding (package
// clunkyard, which the arena provides). It sweeps a medium scan to find a
// robot, or points it where a passive contact was heard, narrows the scan
// to hold it, leads it with shots whose power suits the range, fires only
// when the turret is on target, keeps battery back for driving and keeps
// out of the storm.
//
// Bearing is earned: a scan contact's bearing is off by up to its
// bearingError (degrees, a fixed share of the arc), so a wide beam finds a
// robot and a narrow one aims at it. The robot narrows the beam, holds it
// on the target and aims along a line fitted through its recent sightings.
// Contacts within 120 units are exact.
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
//   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
//   from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import clunkyard.*;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.ToDoubleFunction;

public final class Bot extends Robot {
  /** Enough to turn against the brake in the force rulesets. */
  private static final double SEARCH_TURN = 0.33;
  private static final double SEARCH_ARC = 40; // degrees: sweeps the circle in half a second
  private static final double HOLD_WIDTH = 60; // units across at the target: it stays in the beam from tick to tick
  private static final double MIN_ARC = 6; // degrees: the narrowest the beam holds
  private static final int WINDOW = 12; // ticks of sightings the aim averages
  private static final int SIGHTINGS = 8; // narrow sightings in the window before a shot
  private static final int LOOK_EVERY = 40; // ticks between looks around the target, for a nearer robot
  private static final int REACQUIRE = 10; // ticks to look again where a lost target was

  /** A line through recent sightings: where the robot is now, how fast it moves, and how many sightings it rests on. */
  private record Fit(Vec3 position, Vec3 velocity, int count) {}

  private boolean free; // flying free, robots may be above or below
  private int dodges, dodgeEnd; // dodges taken, and the tick the current one ends
  private int slot; // the last target's slot, for looking again where it was lost

  /** A turn or pitch-turn control towards an angle, full at 45 degrees off. */
  private double steer(double towards, double current) {
    return angles().clamp(angles().angleDiff(towards, current) / 45);
  }

  /**
   * A straight line through the narrowest of the last WINDOW ticks'
   * sightings: where the robot is now and how fast it moves. Each sighting
   * is off by its bearingError; the line averages that out.
   */
  private Optional<Fit> fit(int slot, int tick) {
    var recent = tracker().history(slot).stream().filter(s -> s.tick() > tick - WINDOW).toList();
    if (recent.size() < 2) return Optional.empty();
    double best = recent.stream().mapToDouble(Sighting::bearingError).min().getAsDouble();
    var seen = recent.stream().filter(s -> s.bearingError() <= 1.5 * best).toList(); // the narrow ones
    int n = seen.size();
    if (n < 2) return Optional.empty();
    double t0 = seen.stream().mapToDouble(Sighting::tick).sum() / n;
    double spread = seen.stream().mapToDouble(s -> (s.tick() - t0) * (s.tick() - t0)).sum();
    double[] position = new double[3], velocity = new double[3];
    for (int axis = 0; axis < 3; axis++) {
      int a = axis;
      ToDoubleFunction<Sighting> at = s -> a == 0 ? s.position().x() : a == 1 ? s.position().y() : s.position().z();
      double mean = seen.stream().mapToDouble(at).sum() / n;
      double slope = seen.stream().mapToDouble(s -> (s.tick() - t0) * (at.applyAsDouble(s) - mean)).sum() / spread;
      position[a] = mean + slope * (tick - t0);
      velocity[a] = slope * match().tickRate();
    }
    return Optional.of(new Fit(new Vec3(position[0], position[1], position[2]), new Vec3(velocity[0], velocity[1], velocity[2]), n));
  }

  @Override
  protected void onStart() {
    free = match().is3d() && arena().gravity().z() == 0;
  }

  @Override
  protected void onTurn() {
    Me me = me();
    Vec3 position = me.position();
    int tick = clock().tick();
    drive().forces(Drive.brake(0.2));
    var nearest = sensors().nearest();
    var heard = tracker().cue(); // the freshest passive contact, for a few seconds
    var lastSeen = slot != 0 ? tracker().lastSeen(slot) : OptionalInt.empty(); // empty until a target has been seen
    if (nearest.isPresent()) {
      Contact target = nearest.get();
      slot = target.slot();
      double throttle = target.range() > 350 ? 0.7 : target.range() < 200 ? -0.4 : 0.2;
      drive().set(throttle, steer(target.bearing(), me.heading()), steer(target.elevation(), me.pitch()));
      // Halve the beam each tick until it is HOLD_WIDTH across at the target:
      // narrow enough for a precise bearing, wide enough to keep it in.
      double hold = Math.max(MIN_ARC, Math.toDegrees(2 * Math.atan2(HOLD_WIDTH / 2, target.range())));
      // In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks,
      // widen it for a tick: a nearer robot it finds becomes the target.
      double arc = tick % LOOK_EVERY == 0 ? SEARCH_ARC : Math.max(hold, Math.min(SEARCH_ARC, scanner().last().arc() / 2));
      scanner().set(target.bearing(), arc, target.elevation());
      // The heaviest shot that still covers the range in about a second.
      double power = gun().powerForFlightTime(target.range(), 1);
      // An exact contact: gun().lead is the intercept if the target holds
      // course. A noisy one: lead the fitted line, and fire only once it
      // rests on enough sightings.
      Optional<Solution> aim;
      boolean ready = true;
      if (target.bearingError() > 0) {
        var fit = fit(target.slot(), tick);
        aim = fit.flatMap(f -> gun().leadPoint(f.position(), f.velocity(), power));
        ready = fit.isPresent() && fit.get().count() >= SIGHTINGS;
      } else aim = gun().lead(target, power);
      double heading = aim.map(Solution::heading).orElse(target.bearing());
      double pitch = aim.map(Solution::pitch).orElse(target.elevation());
      gun().aim(heading, pitch);
      Direction now = gun().aimNow();
      boolean onTarget = Math.abs(angles().angleDiff(heading, now.heading())) < 3 && Math.abs(pitch - now.pitch()) < 3;
      if (ready && onTarget && gun().canFire(power, 10)) gun().fire(power);
      status().set(ready ? "TRACK" : "NARROW", target.range());
    } else if (lastSeen.isPresent() && tick - lastSeen.getAsInt() <= REACQUIRE) {
      // Lost it: look again where it was last seen, widening the beam, before searching.
      Vec3 where = tracker().position(slot).orElse(position);
      double bearing = angles().bearingTo(position, where);
      double elevation = free ? angles().elevationTo(position, where) : 0;
      drive().set(0.5, steer(bearing, me.heading()), steer(elevation, me.pitch()));
      scanner().set(bearing, Math.min(SEARCH_ARC, scanner().last().arc() * 2), elevation);
      status().set("REACQUIRE");
    } else if (heard.isPresent()) {
      // Head for where it was heard, sweeping an arc that still reaches
      // it across the bearing's error (and, flying free, the elevation's).
      Cue cue = heard.get();
      double bearing = angles().bearingTo(position, cue.position()), distance = angles().distance(position, cue.position());
      double elevation = free ? angles().elevationTo(position, cue.position()) : 0;
      double error = Math.min(cue.bearingError(), 30);
      double across = (tick % 5 - 2) / 2.0 * error;
      double up = free ? ((tick / 5) % 3 - 1) * error / 2 : 0;
      drive().set(0.7, steer(bearing, me.heading()), steer(elevation, me.pitch()));
      scanner().set(bearing + across, scanner().arcForReach(distance + 100), elevation + up);
      status().set("CUE", distance);
    } else {
      drive().set(0.5, SEARCH_TURN, steer(0, me.pitch()));
      // Sweeps the circle in half a second, flying free at three heights in turn.
      double pitch = free ? ((tick / 10) % 3 - 1) * 35 : 0;
      scanner().set(angles().normalize(scanner().last().heading() + 36), SEARCH_ARC, pitch);
      status().set("SEARCH");
    }
    // A simple dodge: vary it, or predict the shooter.
    if (tick >= dodgeEnd && sensors().cues().stream().anyMatch(c -> c.source() == Source.MUZZLE)) {
      dodges++;
      dodgeEnd = tick + 10;
    }
    if (tick < dodgeEnd) {
      // Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
      boolean left = dodges % 2 == 1;
      if (match().control() == Control.FORCES) {
        drive().set(Drive.throttle(0.5));
        drive().forces(Drive.strafe(left ? -1 : 1));
      } else drive().set(1, left ? 1 : -1);
      status().set(left ? "dodge:left" : "dodge:right", dodges);
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
