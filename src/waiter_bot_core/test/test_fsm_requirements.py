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

"""Replay the assessment's seven situations (R1 to R7) against the state machine (T1 to T11)."""
from fsm_helpers import Bot


def test_t1_single_order_no_confirmation():
    """R1: with confirmations off the robot does not wait anywhere."""
    bot = Bot(require_kitchen_confirmation=False, require_table_confirmation=False)
    bot.start('table1').arrive(3)
    assert bot.visited == ['kitchen', 'table1', 'home']
    assert bot.outcome == 'completed' and bot.state == 'idle'


def test_t2_single_order_with_confirmations():
    """R1: with confirmations on, each wait starts a timer that the confirmation stops."""
    bot = Bot()
    bot.start('table1').arrive().confirm('kitchen').arrive().confirm('table1').arrive()
    assert bot.visited == ['kitchen', 'table1', 'home']
    assert bot.outcome == 'completed'


def test_t3_kitchen_timeout_goes_home():
    """R2 and R3a: nobody loads the food, so the robot goes straight home."""
    bot = Bot()
    bot.start('table1').arrive().timeout().arrive()
    assert bot.visited == ['kitchen', 'home']
    assert bot.outcome == 'kitchen_timeout'


def test_t4_table_timeout_returns_food_to_kitchen():
    """R3b: nobody takes the food at the table, so it goes back to the kitchen, then home."""
    bot = Bot()
    bot.start('table1').arrive().confirm('kitchen').arrive().timeout().arrive(2)
    assert bot.visited == ['kitchen', 'table1', 'kitchen', 'home']
    assert bot.outcome == 'partial'
    assert bot.fsm.snapshot().skipped == ('table1',)


def test_t5_cancel_on_the_way_to_the_kitchen():
    """R4a: cancelling before the pickup sends the robot straight home."""
    bot = Bot()
    bot.start('table1').cancel_all().arrive()
    assert bot.visited == ['kitchen', 'home']
    assert bot.outcome == 'cancelled'


def test_t6_cancel_on_the_way_to_a_table():
    """R4b: cancelling with food onboard returns it to the kitchen, then home."""
    bot = Bot()
    bot.start('table1').arrive().confirm('kitchen').cancel_all().arrive(2)
    assert bot.visited == ['kitchen', 'table1', 'kitchen', 'home']
    assert bot.outcome == 'cancelled'


def test_t7_several_orders_all_served():
    """R5: one kitchen visit, every table in order, then home with nothing to return."""
    bot = Bot()
    bot.start('table1', 'table2', 'table3').arrive().confirm('kitchen')
    for table in ('table1', 'table2', 'table3'):
        bot.arrive().confirm(table)
    bot.arrive()
    assert bot.visited == ['kitchen', 'table1', 'table2', 'table3', 'home']
    assert bot.outcome == 'completed'


def test_t8_one_table_ignored_others_served():
    """R6: table1 does not confirm, the others are served, then kitchen, then home."""
    bot = Bot()
    bot.start('table1', 'table2', 'table3').arrive().confirm('kitchen')
    bot.arrive().timeout()
    bot.arrive().confirm('table2')
    bot.arrive().confirm('table3')
    bot.arrive(2)
    assert bot.visited == ['kitchen', 'table1', 'table2', 'table3', 'kitchen', 'home']
    assert bot.outcome == 'partial'
    snapshot = bot.fsm.snapshot()
    assert snapshot.skipped == ('table1',) and snapshot.delivered == ('table2', 'table3')


def test_t9_one_table_cancelled_is_skipped():
    """R7: table2 is cancelled on the way to table1, so it is never visited."""
    bot = Bot()
    bot.start('table1', 'table2', 'table3').arrive().confirm('kitchen')
    bot.cancel_table('table2')
    bot.arrive().confirm('table1')
    bot.arrive().confirm('table3')
    bot.arrive(2)
    assert bot.visited == ['kitchen', 'table1', 'table3', 'kitchen', 'home']
    assert bot.outcome == 'partial'
    assert bot.fsm.snapshot().cancelled == ('table2',)


def test_t10_cancel_while_waiting_at_the_kitchen():
    """R4a: cancelling during the kitchen wait stops the timer and goes home."""
    bot = Bot()
    bot.start('table1').arrive().cancel_all().arrive()
    assert bot.visited == ['kitchen', 'home'] and bot.outcome == 'cancelled'


def test_t11_cancel_while_waiting_at_a_table():
    """R4b: cancelling during the table wait stops the timer and returns the food."""
    bot = Bot()
    bot.start('table1').arrive().confirm('kitchen').arrive().cancel_all().arrive(2)
    assert bot.visited == ['kitchen', 'table1', 'kitchen', 'home']
    assert bot.outcome == 'cancelled'
