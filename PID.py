import time
from collections import deque


class PIDController(object):
    """A software PID controller.

    Parameters:
        proportional (float): :math:`K_p` (no unit).
        integral_time (float): :math:`T_i = K_p/K_i`, in seconds (or
            samples).
        derivative_time (float): :math:`T_d = K_d/K_p`, in seconds (or
            sample).
        vmin (float): minimum value for the output.
        vmax (float): maximum value for the output.

    Attributes:
        setpoint (float): Setpoint value.
        proportional (float): Proportional gain :math:`K_p` (no unit).
        integral (float): Integral gain :math:`K_i` (1/second).
        derivative (float): Derivative gain :math:`K_d` (seconds).
        integral_limit (float): Limit integral value to
            :math:`\pm \mathrm{integral_limit} / {\Delta}t` to prevent
            windup.
        derivative_limit (float): Limit derivative value to
            :math:`\pm \mathrm{derivative_limit} / {/Delta}t` to prevent
            spikes.  Set derivative to 0.0 if derivative is not in the
            range.
        anti_windup (float): Soft integrator, 1.0 to disable,
            recommended range [0.005-0.25].
        proportional_on_pv (bool): Compute proportional gain based on
            the process variable.

    Note:
        The controller follows either the ideal form

        .. math::

            u(t) = K_p e(t) + K_i \int_0^t e(t)dt + K_d \frac{d}{dt}e(t),

        or the standard form

        .. math::

            u(t) = K_p \left(e(t) + \frac{1}{T_i} \int_0^t e(t)dt +
                T_d \frac{d}{dt} e(t) \right).

        Two anti-windup strategies are implemented:
        - a soft integrator `anti_windup` and
        - an integral limitor `integral_limit`.

        The derivative value is computed from the average value of the previous
        five :math:`\Delta E / \Delta t` values to avoid spikes upon large
        setpoint changes.

    Reference:
        - http://en.wikipedia.org/wiki/PID_controller
        - http://www.mstarlabs.com/apeng/techniques/pidsoftw.html

    """
    def __init__(self, 
                 proportional=2.0, integral_time=0.0, derivative_time=0.0,
                 vmin=0.0, vmax=100.0):
        self.proportional = proportional
        self.integral_time = integral_time
        self.derivative_time = derivative_time
        self.vmin = vmin
        self.vmax = vmax
        self.setpoint = 0.0
        self.integral_limit = None
        self.derivative_limit = None
        self.anti_windup = 0.25
        self.proportional_on_pv = False

        self._old_derivative = deque(5 * [0.0], 5)
        self._old_input = 0.0
        self._old_error = 0.0
        self._integral = 0.0
        self._prev_time = time.time()

    def __repr__(self):
        return "".join(
            ["%s(",
             "proportional=%r, integral_time=%r, derivative_time=%r, ",
             "vmin=%r, vmax=%r)"]) % \
                (self.__class__.__name__,
                 self.proportional, self.integral_time, self.derivative_time,
                 self.vmin, self.vmax)

    @property
    def integral_time(self):
        return (0.0 if self.integral == 0.0
                else self.proportional / self.integral)

    @integral_time.setter
    def integral_time(self, integral_time):
        self.integral = (0.0 if integral_time == 0.0
                         else self.proportional / integral_time)

    @property
    def derivative_time(self):
        return self.derivative / self.proportional

    @derivative_time.setter
    def derivative_time(self, derivative_time):
        self.derivative = self.proportional * derivative_time

    def reset(self):
        """Reset time to now."""
        self._prev_time = time.time()

    def compute_output(self, measure, now=None):
        """Compute next output.

        Parameters:
            measure (float): Process value.
            now (float, optional): Time in s or time.time() if the value
                is omitted.

        Returns:
            output (float): Output value.

        """
        error = self.setpoint - measure
        # print ("error = ", error)

        if now is None:
            now = time.time()
        dt = now - self._prev_time
        p = self.proportional * (measure if self.proportional_on_pv else error)
        if dt > 0.0:
            i = self.integral * self._integral * dt
            derivative = (error - self._old_error) / dt
            if self.derivative_limit:
                derivative_limit = self.derivative_limit / dt
                if not (-derivative_limit < derivative < derivative_limit):
                    derivative = 0.0
            self._old_derivative.append(derivative)
            slope = sum(self._old_derivative) / len(self._old_derivative)
            d = self.derivative * slope
        else:
            i = d = 0.0

        self._prev_time = now
        self._old_input = measure
        self._old_error = error

        u = p + i + d
        if u > self.vmax:
            u = self.vmax
            self._integral += self.anti_windup * error
        elif u < self.vmin:
            u = self.vmin
            self._integral += self.anti_windup * error
        else:
            self._integral += error

        if self.integral_limit:
            integral_limit = self.integral_limit / dt
            if self._integral > integral_limit:
                self._integral = integral_limit
            elif self._integral < -integral_limit:
                self._integral = -integral_limit

        return u


# def _test_system():
#     """Test PID algorithm."""
#     import scipy.signal as sig
#     # transfer function in s-space describes sys
#     tf = sig.tf2ss([10], [100, 10])
#     times = np.arange(1, 200, 5.0)
#     #step = 1 * np.ones(len(times))
#     # initialize PID
#     pid = PidController(2.0, 10.0, 0.0)
#     pid.anti_windup = 0.2
#     pid.vmin, pid.vmax = -200.0, 200.0
#     pid.setpoint = 50.0
#     pid._prev_time = 0.0
#     sysout = [0.0]
#     pidout = [0.0]
#     real_time = [0.0]
#     for time in times:
#         real_time.append(time)
#         pidout.append(pid.compute_output(sysout[-1], real_time[-1]))
#         t, sysout, xout = sig.lsim(tf, pidout, real_time)

#     fig = plt.figure()
#     ax = fig.add_subplot(111)
#     ax.plot(real_time, sysout, 'r', real_time, pidout, 'b--')
#     plt.show()
