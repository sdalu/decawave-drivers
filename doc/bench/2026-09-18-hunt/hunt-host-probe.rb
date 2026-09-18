# frozen_string_literal: true
require_relative "../gem/test/support/unit"

# Probes for the bug hunt: each one is expected to FAIL against the tree
# as it stands, unless marked otherwise.
class TestProbe < Minitest::Test
    WAIT = 5

    def eventually(what, timeout = WAIT)
        deadline = Time.now + timeout
        until yield
            flunk "timed out waiting for #{what}" if Time.now > deadline
            sleep 0.01
        end
    end

    def setup
        @dw = FakeDevice.new
        @io = DW1000::IO.new(@dw).start
    end

    def teardown
        @io&.stop
    end

    # -- P1: :tx_abort leaves the receiver off when txrx_off raises ---------
    def test_p1_tx_abort_with_a_failing_txrx_off_leaves_the_receiver_off
        @dw.on_txrx_off { raise DW1000::Error::IOError, "SPI transfer failed" }
        assert_nil @io.transmit("no answer", timeout: 0.1)
        eventually("abort processed") { @dw.calls.include?(:txrx_off) }
        sleep 0.2
        # The abort turned the transceiver off and never brought the
        # receiver back: from here on the engine is deaf and nothing in it
        # re-arms.
        assert_equal :rx_start, @dw.calls.last,
                     "receiver left off after a failed abort: #{@dw.calls.inspect}"
    end

    # -- P2: same, when it is rx_start that raises --------------------------
    class RxStartFails < FakeDevice
        def rx_start(mode)
            r = super
            raise DW1000::Error::IOError, "SPI transfer failed" if @boom
            r
        end
        def boom! = @boom = true
    end

    def test_p2_tx_abort_with_a_failing_rx_start_never_retries
        @io.stop
        @dw = RxStartFails.new
        @io = DW1000::IO.new(@dw).start
        @dw.boom!
        assert_nil @io.transmit("no answer", timeout: 0.1)
        eventually("abort processed") { @dw.calls.count(:rx_start) >= 2 }
        sleep 0.3
        # A single failed re-arm is never retried and no later event
        # re-arms either: count the successful arms.
        n = @dw.calls.count(:rx_start)
        @dw.fire([ :rx_timeout, 0 ])
        @dw.fire([ :rx_error, 0 ])
        sleep 0.2
        assert_operator @dw.calls.count(:rx_start), :>, n,
                        "nothing ever re-arms the receiver after a failed" \
                        " abort: #{@dw.calls.inspect}"
    end

    # -- P3: stop() talks to the chip while the command thread still is ----
    def test_p3_stop_touches_the_device_concurrently_with_a_stuck_command
        inside = Queue.new
        left   = Queue.new
        @dw.on_read_temp_vbat do
            inside.push(true)
            # A device call holds the GVL and cannot be killed part way:
            # #process_event and #tx_send do their SPI with it held.
            Thread.handle_interrupt(Object => :never) { sleep 2.0 }
            left.push(true)
            [ 23, 33 ]
        end
        @dw.fire_rx_ok("x")
        inside.pop
        t0 = Process.clock_gettime(Process::CLOCK_MONOTONIC)
        @io.stop
        dt = Process.clock_gettime(Process::CLOCK_MONOTONIC) - t0
        assert_operator dt, :>=, 1.9,
                        "stop() issued txrx_off while the command thread was" \
                        " still inside a device call (returned after #{dt}s)"
    end
end
