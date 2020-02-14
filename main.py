"""
INJAZ RoboMart — Mobile Vending Robot
Team project: autonomous robot that navigates a space and dispenses
on-demand supplies (medications, snacks, essentials).

Modules:
  - Inventory management
  - Robot navigation (grid-based)
  - Obstacle detection (ultrasonic sensor simulation)
  - Customer ordering interface
  - Dispensing & transaction log
  - Battery monitoring
"""

import math
import time
import random
from collections import deque

# ── Inventory ─────────────────────────────────────────────────────────────

class Product:
    def __init__(self, pid, name, price_aed, stock, category):
        self.pid      = pid
        self.name     = name
        self.price    = price_aed
        self.stock    = stock
        self.category = category
        self.sold     = 0

    def __repr__(self):
        return f"[{self.pid}] {self.name:<20} AED {self.price:.2f}  stock={self.stock}"


class Inventory:
    def __init__(self):
        self.products = {}

    def add(self, product):
        self.products[product.pid] = product

    def get(self, pid):
        return self.products.get(pid)

    def dispense(self, pid, qty=1):
        p = self.products.get(pid)
        if p is None:
            return False, "Product not found"
        if p.stock < qty:
            return False, f"Insufficient stock (have {p.stock}, need {qty})"
        p.stock -= qty
        p.sold  += qty
        return True, p

    def low_stock(self, threshold=3):
        return [p for p in self.products.values() if p.stock <= threshold]

    def display(self):
        print("\n  ── RoboMart Inventory ──────────────────────────────")
        cats = {}
        for p in self.products.values():
            cats.setdefault(p.category, []).append(p)
        for cat, items in cats.items():
            print(f"\n  [{cat}]")
            for p in items:
                bar = "█" * p.stock + "░" * max(0, 10 - p.stock)
                print(f"    {p}  {bar}")
        print()

    def sales_report(self):
        print("\n  ── Sales Report ────────────────────────────────────")
        total_revenue = 0
        for p in sorted(self.products.values(), key=lambda x: -x.sold):
            rev = p.sold * p.price
            total_revenue += rev
            print(f"    {p.name:<20} sold={p.sold}  revenue=AED {rev:.2f}")
        print(f"\n    Total revenue: AED {total_revenue:.2f}")


# ── Robot Navigation ──────────────────────────────────────────────────────

class RoboMart:
    GRID_W, GRID_H = 10, 8   # metres

    def __init__(self, start=(0, 0)):
        self.x, self.y   = start
        self.heading     = 0        # degrees, 0 = East
        self.speed_ms    = 0.5      # m/s
        self.battery_pct = 100.0
        self.obstacles   = set()
        self.log         = []
        self.transactions = []

    # -- Sensors --
    def ultrasonic_scan(self):
        """Simulate ultrasonic sensor reading (cm to nearest obstacle)."""
        reading = random.uniform(30, 200)   # cm
        if random.random() < 0.15:          # 15% chance of close obstacle
            reading = random.uniform(5, 25)
        return round(reading, 1)

    def battery_voltage(self):
        """12V battery pack, linear discharge model."""
        return round(9.0 + (self.battery_pct / 100) * 3.0, 2)

    def _drain(self, dist_m):
        self.battery_pct = max(0, self.battery_pct - dist_m * 0.4)

    # -- Movement --
    def move_to(self, tx, ty):
        """Move robot to target coordinates with obstacle check."""
        dist  = math.sqrt((tx - self.x)**2 + (ty - self.y)**2)
        angle = math.degrees(math.atan2(ty - self.y, tx - self.x))

        # obstacle check before moving
        sensor = self.ultrasonic_scan()
        if sensor < 20:
            msg = f"  ⚠ Obstacle at {sensor}cm — rerouting from ({self.x},{self.y})"
            print(msg)
            self.log.append(msg)
            # simple detour: shift perpendicular then retry
            tx, ty = ty, tx   # swap as basic detour demo
            dist   = math.sqrt((tx - self.x)**2 + (ty - self.y)**2)

        travel_time = dist / self.speed_ms
        self._drain(dist)
        self.x, self.y = round(tx, 1), round(ty, 1)
        self.heading   = angle % 360
        return round(travel_time, 2)

    def go_home(self):
        t = self.move_to(0, 0)
        print(f"  Returning to dock — travel time {t:.1f}s  battery={self.battery_pct:.1f}%")

    def status(self):
        print(f"\n  RoboMart Position : ({self.x}, {self.y}) m")
        print(f"  Heading           : {self.heading:.1f}°")
        print(f"  Battery           : {self.battery_pct:.1f}%  ({self.battery_voltage()} V)")
        print(f"  Speed             : {self.speed_ms} m/s")

    # -- Ordering & Dispensing --
    def serve_customer(self, location, pid, qty, inventory, customer_name="Customer"):
        print(f"\n  ── Order Request ───────────────────────────────────")
        print(f"  Customer : {customer_name}")
        print(f"  Location : {location}")
        print(f"  Item PID : {pid}  Qty: {qty}")

        # navigate to customer
        t = self.move_to(*location)
        print(f"  Navigated to {location} in {t:.1f}s")

        # dispense
        ok, result = inventory.dispense(pid, qty)
        if not ok:
            print(f"  Dispense FAILED: {result}")
            return None

        total = result.price * qty
        tx = {
            "customer": customer_name,
            "product": result.name,
            "qty": qty,
            "total_aed": total,
            "location": location
        }
        self.transactions.append(tx)
        print(f"  Dispensed  : {qty}× {result.name}")
        print(f"  Charged    : AED {total:.2f}")
        print(f"  Stock left : {result.stock}")
        return tx

    def transaction_log(self):
        print("\n  ── Transaction Log ─────────────────────────────────")
        grand = 0
        for i, tx in enumerate(self.transactions, 1):
            print(f"  {i}. {tx['customer']:<15} {tx['product']:<20} "
                  f"x{tx['qty']}  AED {tx['total_aed']:.2f}  @ {tx['location']}")
            grand += tx["total_aed"]
        print(f"\n  Session total: AED {grand:.2f}")


# ── Path Planning (BFS on grid) ───────────────────────────────────────────

def bfs_path(grid, start, goal):
    """
    BFS shortest path on a 2D grid.
    grid: 2D list (0=free, 1=obstacle)
    Returns list of (row,col) steps or None if no path.
    """
    rows, cols = len(grid), len(grid[0])
    visited = [[False]*cols for _ in range(rows)]
    parent  = {}
    queue   = deque([start])
    visited[start[0]][start[1]] = True

    dirs = [(-1,0),(1,0),(0,-1),(0,1)]
    while queue:
        r, c = queue.popleft()
        if (r, c) == goal:
            path = []
            while (r, c) != start:
                path.append((r, c))
                r, c = parent[(r, c)]
            path.append(start)
            return list(reversed(path))
        for dr, dc in dirs:
            nr, nc = r+dr, c+dc
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc] and grid[nr][nc] == 0:
                visited[nr][nc] = True
                parent[(nr, nc)] = (r, c)
                queue.append((nr, nc))
    return None   # no path found

def print_path(grid, path):
    display = [row[:] for row in grid]
    for step, (r, c) in enumerate(path):
        display[r][c] = step + 1 if step not in (0, len(path)-1) else ("S" if step == 0 else "G")
    print("\n  Grid path (S=start, G=goal, number=step, X=obstacle):")
    for row in display:
        print("   ", " ".join(str(c).rjust(2) if isinstance(c, int) else f" {c}" for c in row))


# ── Main Demo ─────────────────────────────────────────────────────────────

if __name__ == "__main__":

    # --- Build Inventory ---
    inv = Inventory()
    inv.add(Product("P01", "Panadol Extra",    3.50,  8, "Medicine"))
    inv.add(Product("P02", "Strepsils",        5.00,  6, "Medicine"))
    inv.add(Product("P03", "Hand Sanitizer",   7.00, 10, "Hygiene"))
    inv.add(Product("P04", "Face Mask (5pk)",  4.00,  5, "Hygiene"))
    inv.add(Product("P05", "Water 500ml",      1.50, 15, "Drinks"))
    inv.add(Product("P06", "Energy Bar",       4.50,  7, "Snacks"))
    inv.add(Product("P07", "Chewing Gum",      2.00,  2, "Snacks"))  # low stock
    inv.add(Product("P08", "Tissues",          2.50,  9, "Hygiene"))

    inv.display()

    # --- Robot Init ---
    robot = RoboMart(start=(0, 0))
    print("=== RoboMart System Startup ===")
    robot.status()

    # --- Serve Customers ---
    print("\n=== Customer Orders ===")
    robot.serve_customer((3.0, 2.0), "P01", 2, inv, "Ahmed Al Mansoori")
    robot.serve_customer((6.5, 4.0), "P03", 1, inv, "Sara Khalid")
    robot.serve_customer((1.5, 7.0), "P05", 3, inv, "Mohammed Hassan")
    robot.serve_customer((8.0, 3.5), "P02", 1, inv, "Layla Ibrahim")

    # --- Low Stock Alert ---
    print("\n=== Low Stock Alerts ===")
    low = inv.low_stock(threshold=4)
    if low:
        for p in low: print(f"  ⚠ LOW: {p}")
    else:
        print("  All items adequately stocked.")

    # --- Path Planning Demo ---
    print("\n=== Path Planning (BFS) ===")
    grid = [
        [0,0,0,0,0,0,0,0],
        [0,1,1,0,0,1,0,0],
        [0,0,0,0,1,1,0,0],
        [0,0,1,0,0,0,0,0],
        [0,0,1,1,0,0,1,0],
        [0,0,0,0,0,0,0,0],
    ]
    path = bfs_path(grid, (0,0), (5,7))
    if path:
        print(f"  Path found: {len(path)} steps")
        print_path(grid, path)
    else:
        print("  No path found.")

    # --- End of Session ---
    robot.go_home()
    robot.status()
    robot.transaction_log()
    inv.sales_report()
