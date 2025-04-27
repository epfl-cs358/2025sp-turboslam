import * as TabsPrimitive from "@radix-ui/react-tabs";
import { ComponentProps } from "react";

export const MyTabs = TabsPrimitive.Root;
export const MyTabsContent = TabsPrimitive.Content;

export function MyTabsList(props: ComponentProps<typeof TabsPrimitive.List>) {
  return (
    <TabsPrimitive.List
      {...props}
      className={`flex justify-center gap-2 ${props.className ?? ""}`}
    />
  );
}

export function MyTabsTrigger({
  className = "",
  ...props
}: ComponentProps<typeof TabsPrimitive.Trigger>) {
  return (
    <TabsPrimitive.Trigger
      {...props}
      className={`button px-4 py-2 font-bold text-black [--zoom:10] ${className}`}
    />
  );
}
