"""Focused-window key-down/up input. No global keyboard hook or motor I/O."""


class HeldKeys:
    directions = {'w': 'x+', 's': 'x-', 'a': 'y+', 'd': 'y-', 'r': 'z+', 'f': 'z-'}

    def __init__(self):
        self.held = set()
        self.blocked = set()
        self.revision = 0
        self.action = None
        self.scale = 1.

    def stop(self):
        self.blocked.update(self.held)
        self.held.clear()
        self.revision += 1

    def press(self, key):
        key = key.lower()
        if self.action is not None:
            return
        if key in ('return', 'kp_enter', 'q', 'escape'):
            self.stop()
            self.action = 'save' if key in ('return', 'kp_enter') else 'quit'
        elif key == 'space':
            self.stop()
        elif key in ('1', '2', '3'):
            self.scale = {'1': .5, '2': 1., '3': 2.}[key]
            self.revision += 1
        elif key in self.directions and key not in self.blocked and key not in self.held:
            self.held.add(key)
            self.revision += 1

    def release(self, key):
        key = key.lower()
        self.blocked.discard(key)
        if key in self.held:
            self.held.remove(key)
            self.revision += 1

    def current(self):
        if self.action:
            return self.action, self.revision
        return (self.directions[next(iter(self.held))] if len(self.held) == 1 else None,
                self.revision)


class JogWindow:
    def __init__(self, speed_mm_s):
        self.speed = speed_mm_s
        self.keys = HeldKeys()
        self.root = None
        self.pending_releases = {}
        self.active = False

    def __enter__(self):
        import tkinter as tk
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            raise RuntimeError('XYZ 조절창을 열 수 없습니다. 노트북의 그래픽 데스크톱 터미널에서 실행하세요.') from exc
        self.root.withdraw()  # Show only after the automatic approach finishes.
        self.root.title('OMX XYZ - WASD / R F')
        self.root.geometry('640x350')
        self.root.resizable(False, False)
        self.text = tk.StringVar(value='자동 접근 중 / Automatic approach...')
        tk.Label(self.root, text='W / S : X+ / X-     A / D : Y+ / Y-\n'
                 'R / F : Z+ (UP) / Z- (DOWN)', font=('Sans', 17)).pack(pady=15)
        tk.Label(self.root, text='Hold a key to move. Release to stop.\n'
                 '1: slow    2: normal    3: fast\n'
                 'Enter: save    Q / Esc: cancel    Space: stop', font=('Sans', 13)).pack()
        tk.Label(self.root, textvariable=self.text, font=('Sans', 12), wraplength=610).pack(pady=15)
        self.root.bind('<KeyPress>', self._press)
        self.root.bind('<KeyRelease>', self._release)
        self.root.bind('<FocusOut>', lambda event: self.keys.stop())
        self.root.protocol('WM_DELETE_WINDOW', lambda: self.keys.press('q'))
        self.root.update()
        return self

    def __exit__(self, *args):
        if self.root is not None:
            self.root.destroy()

    def _press(self, event):
        key = event.keysym.lower()
        pending = self.pending_releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)  # X11 repeat release/press pair.
        if self.active:
            self.keys.press(key)

    def _release(self, event):
        key = event.keysym.lower()
        pending = self.pending_releases.pop(key, None)
        if pending is not None:
            self.root.after_cancel(pending)
        def finish():
            self.pending_releases.pop(key, None)
            self.keys.release(key)
        self.pending_releases[key] = self.root.after_idle(finish)

    def activate(self):
        self.root.update()  # Discard input made during the approach.
        self.keys.stop()
        self.active = True
        self.root.deiconify()
        self.root.focus_set()

    def poll(self):
        self.root.update()
        return self.keys.current()

    def status(self, xyz, note=''):
        self.text.set(f'목표 XYZ: {xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f} mm\n'
                      f'속도 상한: {self.speed * self.keys.scale:g} mm/s / 이 창을 클릭하고 조절\n{note}')

    def stop(self):
        self.keys.stop()
