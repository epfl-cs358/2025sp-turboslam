// Main App UI that uses custom components
import { useState } from "react";
import { MyButton } from "@/components/ui/button";
import { MyInput } from "@/components/ui/inputbar";
import {
  MyTabs,
  MyTabsList,
  MyTabsTrigger,
  MyTabsContent,
} from "@/components/ui/tabs";
import { MyPanel } from "@/components/ui/panel";
import { MyJoystick } from "@/components/ui/joystick";

export default function App() {
  const [x, setX] = useState("");
  const [y, setY] = useState("");
  const [logs, setLogs] = useState<string[]>([]);

  const log = (text: string) => setLogs((prev) => [...prev, text]);

  return (
    <div
      className="min-h-screen p-6 font-mono text-white [--block-size:48px]"
      style={{
        backgroundImage: "url('/imgs/items/apple-golden.png')",
        backgroundRepeat: "repeat",
        backgroundSize: "48px 48px",
      }}
    >
      <h1 className="text-shadow-xl mb-6 text-center text-2xl">TurboSLAM </h1>

      <MyTabs defaultValue="user" className="w-full">
        <MyTabsList className="mb-4 justify-center">
          <MyTabsTrigger value="user">User Mode</MyTabsTrigger>
          <MyTabsTrigger value="debug">Debug Mode</MyTabsTrigger>
        </MyTabsList>

        <MyTabsContent value="user">
          <div className="mb-4 flex flex-wrap gap-2">
            <MyButton onClick={() => log("Set Origin")}>Set Origin</MyButton>
            <MyButton onClick={() => log("Exploring...")}>Explore</MyButton>
            <MyInput
              placeholder="X"
              value={x}
              onChange={(e) => setX(e.target.value)}
              className="w-24"
            />
            <MyInput
              placeholder="Y"
              value={y}
              onChange={(e) => setY(e.target.value)}
              className="w-24"
            />
            <MyButton onClick={() => log(`GO to (${x}, ${y})`)}>GO</MyButton>
          </div>

          <div className="flex gap-4">
            <MyPanel
              title="GPS Navigation"
              className="min-h-[var(--9-block)] flex-1"
            >
              <p>
                Target: ({x}, {y})
              </p>

              <div className="map relative mt-2 mr-auto ml-auto h-[var(--7-block)] w-[95%] overflow-hidden rounded [--zoom:2]">
                <div className="absolute right-2 bottom-2 scale-50">
                  <MyJoystick
                    onChange={(val) => console.log("Joystick moved:", val)}
                  />
                </div>
              </div>
            </MyPanel>

            <MyPanel title="3D Map" className="min-h-[var(--6-block)] flex-1">
              <p>[Map display placeholder]</p>
            </MyPanel>
          </div>
        </MyTabsContent>

        <MyTabsContent value="debug">
          <MyPanel title="Debug Logs" variant="dark">
            <pre className="h-[var(--4-block)] overflow-y-scroll text-sm text-green-400">
              {logs.join("\n")}
            </pre>
          </MyPanel>
        </MyTabsContent>
      </MyTabs>
    </div>
  );
}
