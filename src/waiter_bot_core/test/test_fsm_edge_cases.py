# Copyright (c) 2026 Pavan Patil
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Edge cases of the state machine: rejected requests, failures and stale events (T12 to T20)."""
from fsm_helpers import Bot
from waiter_bot_core.butler_fsm import Navigate


def test_t12_cancel_of_a_delivered_table_is_rejected():
    """A table that was already served cannot be cancelled, and nothing changes."""
    bot = Bot()
    bot.start('table1', 'table2').arrive().confirm('kitchen').arrive().confirm('table1')
    before = bot.fsm.snapshot()
    assert 'already delivered' in bot.fsm.can_cancel_table('table1')
    assert bot.fsm.cancel_table('table1') == []
    assert bot.fsm.snapshot() == before


def test_t13_all_tables_cancelled_before_pickup_goes_home():
    """Nothing is loaded when every order is cancelled early, so there is no kitchen return."""
    bot = Bot()
    bot.start('table1', 'table2').cancel_table('table1').cancel_table('table2')
    assert bot.state == 'to_home'
    bot.arrive()
    assert bot.visited == ['kitchen', 'home'] and bot.outcome == 'partial'


def test_t14_navigation_failure_is_retried():
    """A failed trip is sent again until the retry budget is used."""
    bot = Bot(nav_max_retries=2)
    bot.start('table1').nav_failed().nav_failed()
    assert bot.visited == ['kitchen', 'kitchen', 'kitchen'] and bot.state == 'to_kitchen'


def test_t15_unreachable_table_is_skipped():
    """After the retries a table that cannot be reached is skipped, and the food returned."""
    bot = Bot(nav_max_retries=1, require_kitchen_confirmation=False)
    bot.start('table1', 'table2').arrive()
    bot.nav_failed().nav_failed()
    assert bot.fsm.snapshot().skipped == ('table1',) and bot.visited[-1] == 'table2'
    bot.arrive().confirm('table2').arrive(2)
    assert bot.visited[-2:] == ['kitchen', 'home'] and bot.outcome == 'partial'


def test_t15b_unreachable_kitchen_goes_home():
    """If the kitchen cannot be reached the orders are skipped and the robot goes home."""
    bot = Bot(nav_max_retries=0)
    bot.start('table1').nav_failed().arrive()
    assert bot.visited == ['kitchen', 'home'] and bot.outcome == 'nav_failed'


def test_t16_unreachable_home_is_an_error():
    """A trip home that keeps failing ends the task in the error state, never silently."""
    bot = Bot(nav_max_retries=0)
    bot.start('table1').arrive().timeout().nav_failed()
    assert bot.state == 'error' and bot.outcome == 'nav_failed'
    assert bot.fsm.check_goal(('table1',)) == ''


def test_t18_confirmation_for_the_wrong_location_is_rejected():
    """Only the location the robot is waiting at can confirm, and the timer keeps running."""
    bot = Bot()
    bot.start('table1').arrive()
    assert 'waiting at kitchen' in bot.fsm.can_confirm('table1')
    assert bot.fsm.confirmed('table1') == [] and bot.timer


def test_t19_goal_validation():
    """Unknown or duplicate tables, an empty list and a second goal are all rejected."""
    bot = Bot()
    assert bot.fsm.check_goal(()) == 'no tables given'
    assert bot.fsm.check_goal(('table1', 'table1')) == 'duplicate table'
    assert bot.fsm.check_goal(('table9',)) == 'unknown table table9'
    bot.start('table1')
    assert bot.fsm.check_goal(('table2',)) == 'busy'


def test_t20_stale_events_are_ignored():
    """A timeout after a confirmation, or a repeated arrival, does nothing."""
    bot = Bot()
    bot.start('table1').arrive().confirm('kitchen')
    assert bot.fsm.timeout() == []
    state = bot.state
    assert bot.fsm.confirmed('kitchen') == [] and bot.state == state
    bot.arrive().confirm('table1')
    assert bot.fsm.timeout() == []


def test_return_dwell_waits_before_going_home():
    """With a dwell time the robot pauses at the kitchen after returning food."""
    bot = Bot(kitchen_return_dwell_sec=3.0)
    bot.start('table1').arrive().confirm('kitchen').arrive().timeout().arrive()
    assert bot.state == 'returning_food' and bot.timer
    assert bot.fsm.arrived() == []
    bot.timeout()
    assert bot.fsm.state.value == 'to_home' and bot.visited[-1] == 'home'


def test_snapshot_lists_every_order():
    """Delivered, skipped and cancelled together account for all orders at the end."""
    bot = Bot()
    bot.start('table1', 'table2', 'table3').arrive().confirm('kitchen')
    bot.cancel_table('table3').arrive().confirm('table1').arrive().timeout().arrive(2)
    snapshot = bot.fsm.snapshot()
    assert (snapshot.delivered, snapshot.skipped, snapshot.cancelled) == (
        ('table1',), ('table2',), ('table3',))
    assert snapshot.pending == () and snapshot.state == 'idle'
    assert bot.fsm.start(('table1',)) == [Navigate('kitchen')]


def test_food_is_not_onboard_after_it_is_returned():
    """Once the food is back at the kitchen the robot goes home empty, and says so."""
    bot = Bot()
    bot.start('table1').arrive().confirm('kitchen').arrive().timeout()
    assert bot.fsm.snapshot().food_onboard
    bot.arrive()
    assert bot.state == 'to_home' and not bot.fsm.snapshot().food_onboard
