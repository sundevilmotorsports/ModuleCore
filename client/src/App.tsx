import { useEffect, useState, useCallback } from 'react'
import { invoke } from '@tauri-apps/api/core'
import { listen } from '@tauri-apps/api/event'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { Separator } from '@/components/ui/separator'
import { ScrollArea } from '@/components/ui/scroll-area'
import { DeviceRow, type Device } from '@/components/DeviceRow'

interface AppState {
  port_name: string
  devices: Device[]
}

export default function App() {
  const [state, setState] = useState<AppState>({ port_name: '', devices: [] })
  const [scanning, setScanning] = useState(false)
  const [selectedDevice, setSelectedDevice] = useState<Device | null>(null);
  const [editingCanId, setEditingCanId] = useState<string>('')
  const [editError, setEditError] = useState<string | null>(null)

  const applyState = useCallback((s: AppState) => {
    setState(s)
  }, [])

  useEffect(() => {
    invoke<AppState>('get_state').then(applyState).catch(console.error)

    const unlisten = listen<AppState>('state-update', (event) => {
      applyState(event.payload)
    })

    return () => { unlisten.then(fn => fn()) }
  }, [applyState])

  async function handleDiscover() {
    setScanning(true)
    try {
      const s = await invoke<AppState>('get_state')
      applyState(s)
    } catch (e) {
      console.error(e)
    } finally {
      setScanning(false)
    }
  }

  const connected = state.port_name !== ''

  return (
    <div className="flex h-screen flex-col bg-background p-5 gap-3">
      {/* Header */}
      <div className="flex items-center justify-between">
        <div className="flex flex-col gap-1">
          <span className="text-[11px] font-bold tracking-[3px] text-primary">
            DEVICES
          </span>
          <div className="flex items-center gap-2">
            <Badge
              variant={connected ? 'default' : 'secondary'}
              className="font-mono text-[10px] px-2 py-0"
            >
              {connected ? '● CONNECTED' : '○ SEARCHING'}
            </Badge>
            {connected && (
              <span className="text-[10px] text-muted-foreground font-mono">
                {state.port_name}
              </span>
            )}
          </div>
        </div>

        <Button
          variant="outline"
          size="sm"
          onClick={handleDiscover}
          disabled={scanning || !connected}
          className="font-mono text-[10px] tracking-[2px] text-muted-foreground hover:text-primary hover:border-primary"
        >
          {scanning ? 'SCANNING...' : 'DISCOVER'}
        </Button>
      </div>

      <Separator />

      <div className="flex gap-6 flex-1 overflow-hidden">
        {/* Device List */}
        <div className="w-80 flex-shrink-0">
          <ScrollArea className="h-[60vh]">
            <div className="flex flex-col gap-1.5 pr-3">
              {state.devices.length === 0 ? (
                <div className="flex items-center justify-center h-20 text-sm text-muted-foreground">
                  {connected ? 'No devices found' : 'Waiting for device...'}
                </div>
              ) : (
                state.devices.map((dev) => (
                  <DeviceRow
                    key={dev.can_id}
                    device={dev}
                    onSelect={(d) => {
                      setSelectedDevice(d)
                      setEditingCanId(String(d.can_id))
                      setEditError(null)
                    }}
                    selected={selectedDevice?.can_id === dev.can_id}
                  />
                ))
              )}
            </div>
          </ScrollArea>
        </div>

        {/* Device Panel */}
        <div className="flex-1">
          <div className="p-4 rounded-md min-h-[200px]">
            {selectedDevice && connected ? (
              <div className="flex items-start gap-3">
                <div className="flex flex-col">
                  <label className="text-[9px] text-muted-foreground">CAN ID</label>
                  <input
                    className="font-mono border px-2 py-1 rounded text-sm"
                    value={editingCanId}
                    onChange={(e) => setEditingCanId(e.target.value)}
                  />
                  {editError && <span className="text-xs text-destructive">{editError}</span>}
                </div>

                <div className="flex items-center gap-2 mt-4">
                  <Button
                    size="sm"
                    onClick={async () => {
                      const raw = editingCanId.trim()
                      if (raw === '') { setEditError('CAN ID cannot be empty'); return }
                      const parsed = Number(raw)
                      if (!Number.isInteger(parsed) || parsed < 0 || parsed > 255) {
                        setEditError('Enter a valid CAN ID')
                        return
                      }
                      setEditError(null)
                      try {
                        await invoke('set_can_id', { oldCanId: selectedDevice.can_id, newCanId: parsed })
                        setState((s) => ({
                          ...s,
                          devices: s.devices.map(d => d.can_id === selectedDevice.can_id ? { ...d, can_id: parsed, can_id_str: String(parsed) } : d)
                        }))
                        setSelectedDevice((sd) => sd ? { ...sd, can_id: parsed, can_id_str: String(parsed) } : sd)
                      } catch (e) {
                        console.error(e)
                        setEditError('Failed to set CAN ID')
                      }
                    }}
                  >
                    SAVE
                  </Button>
                </div>
              </div>
            ) : (
              <div className="text-sm text-muted-foreground">No Device Selected</div>
            )}
          </div>
        </div>
      </div>
    </div>
  )
}
