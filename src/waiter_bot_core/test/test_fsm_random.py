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

"""Random event sequences must never break the machine's rules (T17, fixed seed)."""
import random

from fsm_helpers import Bot, TABLES

EVENTS = ('arrive', 'confirm', 'timeout', 'cancel_all', 'cancel_table', 'nav_failed')


def play(rng, allow_failures):
    """Run one random task and return the bot, checking the invariants after every event."""
    bot = Bot(nav_max_retries=1,
              require_kitchen_confirmation=rng.random() < 0.7,
              require_table_confirmation=rng.random() < 0.7)
    bot.start(*rng.sample(TABLES, rng.randint(1, 3)))
    for _ in range(rng.randint(0, 25)):
        event = rng.choice(EVENTS)
        if event == 'nav_failed' and not allow_failures:
            continue
        before = len(bot.visited)
        if event == 'arrive':
            bot.arrive()
        elif event == 'confirm':
            bot.confirm(rng.choice(('kitchen',) + TABLES))
        elif event == 'timeout':
            bot.timeout() if bot.timer else None
        elif event == 'cancel_all':
            bot.cancel_all()
        elif event == 'cancel_table':
            bot.cancel_table(rng.choice(TABLES))
        else:
            bot.nav_failed()
        if not allow_failures and len(bot.visited) > before and bot.visited[-1] == 'home':
            assert not bot.fsm.food_onboard, 'went home with food onboard'
    return bot


def finish(bot):
    """Let every remaining trip succeed and every wait time out, then check the task ended."""
    for _ in range(40):
        if bot.state in ('idle', 'error'):
            break
        bot.timeout() if bot.timer else bot.arrive()
    assert bot.state in ('idle', 'error'), 'task did not end'
    assert not bot.timer


def test_t17_random_sequences_keep_the_rules():
    """1000 random tasks: one timer at a time, no food carried home, every task ends."""
    rng = random.Random(20261102)
    for _ in range(1000):
        finish(play(rng, allow_failures=False))
    for _ in range(1000):
        finish(play(rng, allow_failures=True))
