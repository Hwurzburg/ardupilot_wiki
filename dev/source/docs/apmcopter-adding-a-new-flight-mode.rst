.. _apmcopter-adding-a-new-flight-mode:

==================================
Adding a New Flight Mode to Copter
==================================

This section covers the basics of how to create a new high level flight mode (i.e. equivalent of Stabilize, Loiter, etc)

As a reference the diagram below provides a high level view of Copter's architecture.

.. image:: ../images/copter-architecture.png
    :target: ../_images/copter-architecture.png
    :width: 450px

#. Pick a name for the new mode and add it to the bottom of the ``Mode::Number`` enum in `mode.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode.h>`__ just like "NEW_MODE" has been added below.

   ::

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ...
        AUTOROTATE =   26,  // Autonomous autorotation
        AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        TURTLE =       28,  // Flip over after crash
        NEW_MODE =     29,  // your new flight mode

        // Mode number 30 reserved for "offboard" for external/lua control.
        ...
    };

   The number 29 is only an example. Choose a number that is not used or reserved (see the comments at the end of the enum), check open pull requests for new modes that may already claim it, and never reuse the number of a mode that has been released. If the mode is only for your own fork, a high unused number avoids clashes with future ArduPilot modes. Ground station tools that use a fixed table of mode names (see the last step) may already have an entry for a number that Copter does not use; pymavlink, for example, still lists 29 as ``RATE_ACRO``, so MAVProxy would show a new mode 29 under that name.

#. Define a new class for the mode in `mode.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode.h>`__.
   It is probably easiest to copy a similar existing mode's class definition and just change the class name (i.e. copy and rename "class ModeStabilize" to "class ModeNewMode").
   The new class should inherit from the Mode class and implement ``mode_number()``, ``run()``, ``name()`` and ``name4()`` and optionally ``init()``.

   ::

        class ModeNewMode : public Mode {

        public:
            // inherit constructor
            using Mode::Mode;
            Number mode_number() const override { return Number::NEW_MODE; }

            bool init(bool ignore_checks) override;
            void run() override;

            bool requires_position() const override { return false; }
            bool has_manual_throttle() const override { return true; }
            bool allows_arming(AP_Arming::Method method) const override { return true; };
            bool is_autopilot() const override { return false; }

        protected:
            const char *name() const override { return "New Mode"; }
            const char *name4() const override { return "NEWM"; }
        };

   ``mode_number()`` returns the number added to the enum above.  ``init()`` will be called when the vehicle first switches into this new mode so it should implement any required initialisation.  ``run()`` will be called at the main loop rate (400Hz by default) and should implement any pilot input decoding and then set position and attitude targets (see below).

   ``name()`` is the full mode name. It is used in messages such as pre-arm and mode change failures, and is the name sent to ground stations in the ``AVAILABLE_MODES`` message. ``name4()`` is a four character abbreviation shown on the OSD, other displays and RC telemetry. Logs record only the mode number; log viewers look up the name themselves.

   ``requires_position()``, ``has_manual_throttle()``, ``allows_arming()`` and ``is_autopilot()`` must also be implemented. They control features such as whether the vehicle needs a position estimate to enter the mode and whether it can be armed in it. Other methods with default implementations, such as ``enabled()``, can be overridden; see the Mode class definition at the top of mode.h.

#. Create a new mode_<new flight mode>.cpp file based on a similar mode such as
   `mode_stabilize.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_stabilize.cpp>`__
   or `mode_loiter.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_loiter.cpp>`__.
   This new file should probably implement the ``init()`` method which will be called when the vehicle first enters the mode.  This function should return true if it is OK for the vehicle to enter the mode, false if it cannot.
   Below is an excerpt from `mode_rtl.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_rtl.cpp>`__'s init method that shows how the vehicle cannot enter RTL mode unless the home position has been set.

   ::

        bool ModeRTL::init(bool ignore_checks)
        {
            if (!ignore_checks) {
                if (!AP::ahrs().home_is_set()) {
                    return false;
                }
            }
            // initialise waypoint and spline controller
            wp_nav->wp_and_spline_init_m(speed_ms.get());
            _state = SubMode::STARTING;
            _state_complete = true; // see run() method below
            terrain_following_allowed = !copter.failsafe.terrain;
            ...
            return true;
        }


   Below is an excerpt from `mode_stabilize.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode_stabilize.cpp>`__'s run method that decodes the user's input, then sends new targets to the attitude controller.

   ::

        void ModeStabilize::run()
        {
            // convert pilot input to lean angles
            float target_roll_rad, target_pitch_rad;
            get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());

            // get pilot's desired yaw rate
            float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();

            // code that sets motor spool state omitted

            // call attitude controller
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

            // output pilot's throttle
            attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
        }

#. Add a ``MODE_NEWMODE_ENABLED`` define to `config.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/config.h>`__, following the pattern used by the other modes (e.g. ``MODE_ZIGZAG_ENABLED``), and wrap the new mode's class and code in ``#if MODE_NEWMODE_ENABLED`` so it can be left out of builds for boards with limited flash. The examples in the following steps assume this define exists; ArduPilot builds treat an undefined name in ``#if`` as an error, so if you leave the define out, leave out those ``#if``/``#endif`` lines as well. For a pull request to ArduPilot, also add a matching ``Feature(...)`` entry for the mode in `Tools/scripts/build_options.py <https://github.com/ArduPilot/ardupilot/blob/master/Tools/scripts/build_options.py>`__ (see ``MODE_ZIGZAG``) so it appears in the custom firmware build options.

#. In `Copter.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.h>`__, add ``friend class ModeNewMode;`` to the list of friend classes (search for "friend class ModeTurtle") so the mode can access Copter's private members. Then instantiate the new mode class by searching for "ModeAcro mode_acro" and adding the new mode somewhere below.

   ::

        #if MODE_ACRO_ENABLED
        #if FRAME_CONFIG == HELI_FRAME
            ModeAcro_Heli mode_acro;
        #else
            ModeAcro mode_acro;
        #endif
        #endif
        #if MODE_ALTHOLD_ENABLED
            ModeAltHold mode_althold;
        #endif
        #if MODE_AUTO_ENABLED
            ModeAuto mode_auto;
        #endif
        ...
        #if MODE_NEWMODE_ENABLED
            ModeNewMode mode_newmode;
        #endif

#. In `mode.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode.cpp>`__ add the new mode to the ``mode_from_mode_num()`` function to create the mapping between the mode's number and the instance of the class.

   ::

        // return the static controller object corresponding to supplied mode
        Mode *Copter::mode_from_mode_num(const Mode::Number mode)
        {

            switch (mode) {
        #if MODE_ACRO_ENABLED
                case Mode::Number::ACRO:
                    return &mode_acro;
        #endif

                case Mode::Number::STABILIZE:
                    return &mode_stabilize;
                ...
        #if MODE_NEWMODE_ENABLED
                case Mode::Number::NEW_MODE:
                    return &mode_newmode;
        #endif

#. Add the new mode to the ``modes[]`` list in ``GCS_MAVLINK_Copter::send_available_mode()`` in `GCS_MAVLink_Copter.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/GCS_MAVLink_Copter.cpp>`__ (wrapped in the same ``#if`` as the mode if it can be compiled out) so that ground stations which use the ``AVAILABLE_MODES`` message list it. Also add it to the matching list in ``Copter::get_available_mode_enabled_mask()`` in `mode.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode.cpp>`__. The mode is still listed without this, but if it overrides ``enabled()`` or is blocked with :ref:`FLTMODE_GCSBLOCK<copter:FLTMODE_GCSBLOCK>`, a change in whether it can be selected will not be announced to the ground station.

#. Add the new flight mode to the list of valid ``@Values`` for the ``FLTMODE1`` parameter in `Parameters.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Parameters.cpp>`__ (search for "FLTMODE1"). ``FLTMODE2`` to ``FLTMODE6`` copy their values from ``FLTMODE1``. This is parameter documentation (metadata) only: once committed to master, it causes the new mode to appear in the list of valid values for these parameters in ground stations that use the parameter metadata.
   Note that even before being committed to master, a user can setup the new flight mode to be activated from the transmitter's flight mode switch by directly setting the FLTMODE1 (or FLTMODE2, etc) parameters to the number of the new mode.

   ::

        // @Param: FLTMODE1
        // @DisplayName: Flight Mode 1
        // @Description: Flight mode when pwm of Flightmode channel(FLTMODE_CH) is <= 1230
        // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,9:Land,11:Drift,13:Sport,14:Flip,15:AutoTune,16:PosHold,17:Brake,18:Throw,19:Avoid_ADSB,20:Guided_NoGPS,21:Smart_RTL,22:FlowHold,23:Follow,24:ZigZag,25:SystemID,26:Heli_Autorotate,27:Auto RTL,28:Turtle,29:New Mode
        // @User: Standard
        GARRAY(flight_modes, 0, "FLTMODE1", (uint8_t)FLIGHT_MODE_1),

#. If it should be possible to stop ground stations from selecting the new mode with :ref:`FLTMODE_GCSBLOCK<copter:FLTMODE_GCSBLOCK>`, add it to the end of the ``mode_list`` in ``Copter::gcs_mode_enabled()`` in `mode.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode.cpp>`__, and add a matching ``@Bitmask{Copter}`` line to the parameter's documentation in `AP_Vehicle.cpp <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Vehicle/AP_Vehicle.cpp>`__. A mode's position in the list is its bit number, so always add to the end.

#. Add the new mode to the ``COPTER_MODE`` enum in `mavlink/ardupilotmega.xml <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml>`__ and submit a PR to `pymavlink <https://github.com/ArduPilot/pymavlink>`__ adding it to the ``mode_mapping_acm`` table in `mavutil.py <https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py>`__. MAVProxy does not use the ``AVAILABLE_MODES`` message and takes its mode names from this table, so the new mode will appear as unknown in MAVProxy (or under a stale name, see the first step) until it is updated. Until then, MAVProxy can still select the mode by number, e.g. ``mode 29``.

   `QGroundControl <https://github.com/mavlink/qgroundcontrol>`__ requests the list of modes from the vehicle with ``AVAILABLE_MODES``, so it shows the new mode by name once it has been added to ``send_available_mode()`` as above. Its hard-coded list of Copter modes in ``ArduCopterFirmwarePlugin`` is only a fallback for older firmware, so adding the new mode there is optional. Mission Planner takes its list of modes from the :ref:`FLTMODE1<copter:FLTMODE1>` parameter metadata, so it needs no changes beyond the ``@Values`` update above. Note that for vehicles running a stable release, Mission Planner uses the metadata for that release rather than master's, and it re-downloads metadata at most once every 7 days, so the new mode may not appear straight away. When testing, you can add the mode to the cached ``<Vehicle>.apm.pdef.xml`` files in Mission Planner's data directory, though they may be overwritten by a later download.
